//
// Created by jbs on 20. 6. 5..
//

#include <atypical_planner/PlannerCore.h>


using namespace Planner;

/**
 * @brief Outputs Vector form for logging
 * @return
 */
VectorXf Corridor::getPretty() {

    VectorXf dataLine(6);
    dataLine(0) = t_start;
    dataLine(1) = t_end;
    dataLine(2) = xl;
    dataLine(3) = xu;
    dataLine(4) = yl;
    dataLine(5) = yu;
    return dataLine;
}

/**
 * @brief Outputs Matrix form for logging
 * @return
 */
MatrixXf MPCResultTraj::getPretty(double t_stamp) {
    if(ts.size()) {
        int N = ts.size();
        MatrixXf data(5,N+1);
        for (int i = 0 ; i <5 ; i++)
            data(i,0) = t_stamp;

        for (int i = 1 ; i < N+1 ; i ++){
            data(0,i) = ts[i-1];
            data(1,i) = xs[i-1].x;
            data(2,i) = xs[i-1].y;
            data(3,i) = us[i-1].alpha;
            data(4,i) = us[i-1].delta;
        }
        return data;
    }
}

/**
 * @brief evaluate state by interpolation
 * @param t
 * @return
 */
CarState MPCResultTraj::evalX(double t) {
    vector<double> xSet(xs.size());
    vector<double> ySet(xs.size());
    // TDOO v, theta?
    for(uint i = 0 ; i < xs.size(); i++){
        xSet[i] = xs[i].x;
        ySet[i] = xs[i].y;
    }

    CarState xy;
    xy.x = interpolate(ts,xSet,t,true);
    xy.y = interpolate(ts,ySet,t,true);
    return xy;
}


CarInput MPCResultTraj::evalU(double t) {

    vector<double> aSet(us.size());
    vector<double> dSet(us.size());
    // TDOO v, theta?
    for(uint i = 0 ; i < us.size(); i++){
        aSet[i] = us[i].alpha;
        dSet[i] = us[i].delta;
    }

    CarInput input;
    input.alpha = interpolate(ts,aSet,t,true);
    input.delta = interpolate(ts,dSet,t,true);
    return input;
}

/**
 * @brief convert lanePath to lane
 * @param lanePath
 */
Lane::Lane(const LanePath& lanePath){

    points.clear();
    widths.clear();

    for (auto & lane :lanePath.lanes){
        for(auto it = lane.laneCenters.begin() ; it!= lane.laneCenters.end(); it++){
            widths.push_back(lane.width);
            points.push_back(Vector2d(it->x,it->y));
        }
    }

}


/**
 * @breif get the points to be considered in a sliding map
 * @param curCarState
 * @param windowOrig
 * @param w width
 * @param h height
 * @return
 */
vector<Vector2d> Lane::slicing(const CarState &curCarState, Vector2d windowOrig, double w, double h, int& startIdx,int& endIdx) {



    // First, we pick the point in points which is closest to current car position
    int StartPointIdx = 1; // this is the index which the current
    double closestDist = 1e+7;
    for (int i = 1 ; i < points.size(); i++){
        Vector2d point = points[i];
        double distance = pow(point(0) - curCarState.x,2) + pow(point(1) - curCarState.y,2);
        Vector2d directionVector = (point - points[i-1]);
        Vector2d headingVector = point - Vector2d(curCarState.x,curCarState.y);
        double innerProduct = (directionVector.dot(headingVector)); // TODO
        if (distance < closestDist and innerProduct >= 0 ){
            closestDist = distance;
            StartPointIdx = i;
        }
    }
    startIdx = StartPointIdx;
//    //Jungwon: Find closest segment point to current car position
//    Vector2d current_point(curCarState.x, curCarState.y);
//    Vector2d a,b,c,pi_min,n;
//    double dist, dist_min;
//    a = points[StartPointIdx-1] - current_point;
//    b = points[StartPointIdx] - current_point;
//    pi_min = a;
//    dist_min = a.norm();
//    dist = b.norm();
//    if(dist_min > dist){
//        pi_min = b;
//        dist_min = dist;
//    }
//    n = (b-a).normalized();
//    c = a - n * a.dot(n);
//    dist = c.norm();
//    if((c-a).dot(c-b) < 0 && dist_min > dist){
//        pi_min = c;
//    }
//    pi_min = pi_min + current_point;

    // Then, let us select the final index
    int EndPointIdx = StartPointIdx;
    for (int i = StartPointIdx ; i < points.size()  ; i++){
        EndPointIdx = i;
        Vector2d point = points[i-1]; // next point
//        ROS_INFO_STREAM("Included points: "<<point.transpose());
        bool isInWindow = (point(0) < windowOrig(0) + w) and
                          (point(0) > windowOrig(0)) and
                          (point(1) < windowOrig(1) + h) and
                          (point(1) > windowOrig(1));

        if (not isInWindow)
            break;

    }

    endIdx = EndPointIdx-1;

//    vector<Vector2d> ret(points.begin()+StartPointIdx,points.begin()+EndPointIdx);
//    ret.insert(ret.begin(), pi_min);
//    return ret;

    return vector<Vector2d>(points.begin()+StartPointIdx,points.begin()+EndPointIdx);
}

/**
 * @brief get nav_msgs/Path of points
 * @param frame_id
 * @return
 */
nav_msgs::Path Lane::getPath(string frame_id) {

    nav_msgs::Path nodeNavPath;
    nodeNavPath.header.frame_id = frame_id.c_str();
    for (auto & lane :points){
            geometry_msgs::PoseStamped aPoint;
            aPoint.pose.position.x = lane(0);
            aPoint.pose.position.y = lane(1);
            nodeNavPath.poses.push_back(aPoint);
    }
    return nodeNavPath;
}

visualization_msgs::MarkerArray Lane::getSidePath(string frame_id) {

    visualization_msgs::Marker baseLane;
    visualization_msgs::MarkerArray sideLanes;

    baseLane.header.frame_id = frame_id;
    baseLane.type = visualization_msgs::Marker::LINE_STRIP;
    baseLane.pose.orientation.w = 1.0;
    baseLane.color.a  = 1.0;
    baseLane.color.r  = 1.0;
    baseLane.color.g  = 1.0;
    baseLane.color.b  = 1.0;
    baseLane.scale.x = 0.1;

    visualization_msgs::Marker leftLane = baseLane; leftLane.ns = "left";
    visualization_msgs::Marker rightLane = baseLane; rightLane.ns = "right";
    int idx = 0 ;

    Matrix2d rot;
    rot << 0, -1,
            1,0;

    for (; idx < points.size() -1; idx ++ ){
        Vector2d dir = (points[idx+1] - points[idx]); dir.normalize();
        Vector2d leftDir = rot*dir;
        Vector2d rightDir = -leftDir;

        geometry_msgs::Point left;
        geometry_msgs::Point right;
        left.x = points[idx](0) + widths[idx]/2*leftDir(0) ;
        left.y = points[idx](1) + widths[idx]/2*leftDir(1) ;
        leftLane.points.push_back(left);

        right.x = points[idx](0) + widths[idx]/2*rightDir(0) ;
        right.y = points[idx](1) + widths[idx]/2*rightDir(1);
        rightLane.points.push_back(right);
    }

    sideLanes.markers.push_back(leftLane);
    sideLanes.markers.push_back(rightLane);

    return sideLanes;
}


/**
 * @brief get visualization_msgs of points
 * @param frame_id
 * @return
 */
visualization_msgs::MarkerArray SmoothLane::getPoints(const string& frame_id) {
    visualization_msgs::MarkerArray markers;
    int i_marker = 0;
    for (auto & point : points){
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.id = i_marker++;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = point(0);
        marker.pose.position.y = point(1);
        marker.pose.position.z = 0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1;
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        markers.markers.emplace_back(marker);
    }
    for(int i = i_marker; i < n_total_markers; i++){
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.id = i;
        marker.action = visualization_msgs::Marker::DELETE;
        markers.markers.emplace_back(marker);
    }
    if(i_marker > n_total_markers){
        n_total_markers = i_marker;
    }
    return markers;
}

vector<Corridor> PlannerBase::getCorridorSeq(double t0, double tf) {

    vector<Corridor> slicedCorridor; bool doPush = false;
    for (auto s : corridor_seq){
        if ( (s.t_start<=t0 and t0 <= s.t_end) and !doPush)
            doPush = true;

        if (s.t_start > tf)
            break;

        if (doPush){
            s.t_start = std::max(s.t_start-t0,0.0);
            s.t_end = s.t_end-t0;
            slicedCorridor.push_back(s);
        }

    }
    corridor_seq[corridor_seq.size()-1].t_end = tf;
    return slicedCorridor;
}


/**
 * @breif update obstaclePathArray so that it is prediction from [tPredictionStart,tPredictionStart+ horizon[
 * @param tSeq_
 * @param rNominal
 */
void PlannerBase::uploadPrediction(VectorXd tSeq_, double rNominal) {

    VectorXf tSeq = tSeq_.cast<float>();
    // TODO predictor should be multiple
    obstaclePathArray.obstPathArray.clear();

    // ver 1
    // TODO check the shape
    for(auto idPredictor : indexedPredictorSet){

        auto predictor = get<1>(idPredictor);
        //predictor.update_predict(); // this will do nothing if observation is not enough
        if (predictor.is_prediction_available()) {
            vector<geometry_msgs::Pose> obstFuturePose = predictor.eval_pose_seq(tSeq);
            ObstaclePath obstPath;
            for (auto obstPose : obstFuturePose) {
                // construct one obstaclePath
                ObstacleEllipse obstE;
                obstE.q = Vector2d(obstPose.position.x, obstPose.position.y);

                // TODO exact tf should be handled / box to elliposid
                obstE.Q.setZero();
                obstE.Q(0, 0) = 1 / pow(predictor.getLastDimensions()(0)/sqrt(2), 2);
                obstE.Q(1, 1) = 1 / pow(predictor.getLastDimensions()(1)/sqrt(2), 2);

                obstPath.obstPath.push_back(obstE);
            }
            obstaclePathArray.obstPathArray.push_back(obstPath);
        }
    }


}
void PlannerBase::log_state_input(double t_cur) {

    string file_name = log_file_name_base + "_state.txt";
    ofstream outfile;
    outfile.open(file_name,std::ios_base::app);
    outfile << t_cur << " "<< cur_state.x << " " << cur_state.y << " " << cur_state.theta << " " << cur_state.v << endl;

    file_name = log_file_name_base + "_input.txt";
    ofstream outfile1;
    outfile1.open(file_name,std::ios_base::app);
    outfile1 <<  t_cur << " "<< getCurInput(t_cur).accel_decel_cmd << " " << getCurInput(t_cur).steer_angle_cmd << endl;
}


void PlannerBase::log_corridor(double t_cur, double tf) {

    // 1. Corridor
    string file_name = log_file_name_base + "_corridor.txt";
    ofstream outfile;
    outfile.open(file_name,std::ios_base::app);
//            outfile << t_cur << endl;
    for (auto corridor : getCorridorSeq(t_cur,tf)){
        outfile<<  t_cur << " "<< corridor.getPretty().transpose() << endl;
    }
}

void PlannerBase::log_mpc(double t_cur) {
    string file_name = log_file_name_base + "_mpc.txt";
    ofstream outfile;
    outfile.open(file_name,std::ios_base::app);
    outfile<< mpc_result.getPretty(t_cur) << endl;
}

/**
 * @brief Get occpuancy
 * @param queryPoint frame = SNU
 * @return true if occupied
 */
bool PlannerBase::isOccupied(Vector2d queryPoint) {
    geometry_msgs::Point queryPoint3;
    queryPoint3.x = queryPoint(0);
    queryPoint3.y = queryPoint(1);
    queryPoint3.z = 0;

    if (!occupancy_grid_utils::withinBounds(localMap.info,queryPoint3)){
//        ROS_WARN("querying point [%f,%f]  is out of bound of occupancy map ", queryPoint3.x,queryPoint3.y );
        return false;
    }


    occupancy_grid_utils::index_t idx = occupancy_grid_utils::pointIndex(localMap.info,queryPoint3);

    return (localMap.data[idx] > OCCUPANCY);
}

bool PlannerBase::isOccupied(Vector2d queryPoint1, Vector2d queryPoint2) {
    geometry_msgs::Point p1;
    p1.x = queryPoint1(0);
    p1.y = queryPoint1(1);
    p1.z = 0;

    geometry_msgs::Point p2;
    p2.x = queryPoint2(0);
    p2.y = queryPoint2(1);
    p2.z = 0;


    occupancy_grid_utils::RayTraceIterRange range = occupancy_grid_utils::rayTrace(localMap.info, p1, p2, true, true);
    auto iter = range.first;
    while(iter != range.second){
        occupancy_grid_utils::Cell cell = *iter;
        occupancy_grid_utils::index_t idx = occupancy_grid_utils::cellIndex(localMap.info, cell);
        if(localMap.data[idx] > OCCUPANCY){
            return true;
        }
        iter++;
    }
    return false;
}