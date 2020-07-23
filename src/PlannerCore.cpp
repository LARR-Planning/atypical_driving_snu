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
 * @brief cut lane points after goal point
 */
void Lane::untilGoal(double goal_x,double goal_y) {
   // Find the closest point toward goal
   int idx = 0;
   Vector2d goal(goal_x,goal_y);
   double deviationMin = 1e+5;
   int newLastIndex = 0;
   for (auto pnt : points){
        double deviation = (pnt - goal).norm();
        if (deviation < deviationMin){
            deviationMin = deviation;
            newLastIndex = idx;
        }
        idx ++;
   }

    points = vector<Vector2d, aligned_allocator<Vector2d>>(points.begin(),points.begin()+newLastIndex);
    widths = vector<double>(widths.begin(),widths.begin()+newLastIndex);

}


/**
 * @breif get the points to be considered in a sliding map
 * @param curCarState
 * @param windowOrig
 * @param w width
 * @param h height
 * @return
 */
vector<Vector2d, aligned_allocator<Vector2d>> Lane::slicing(const CarState &curCarState, Vector2d windowOrig, double w, double h, int& startIdx,int& endIdx) {



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
//    startIdx = StartPointIdx;
    startIdx = StartPointIdx - 1; // (jungwon)

    // Find closest point (jungwon)
    Vector2d current_point = Vector2d(curCarState.x,curCarState.y);
    Vector2d a = points[StartPointIdx-1] - current_point;
    Vector2d b = points[StartPointIdx] - current_point;
    Vector2d n = (b-a).normalized();
    Vector2d start_point = a - n * a.dot(n) + current_point;


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

//    return vector<Vector2d, aligned_allocator<Vector2d>>(points.begin()+StartPointIdx,points.begin()+EndPointIdx);
    vector<Vector2d, aligned_allocator<Vector2d>> ret(points.begin()+StartPointIdx,points.begin()+EndPointIdx);
    ret.insert(ret.begin(), start_point);
    return ret;
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
 * @brief get visualization_msgs of smooth lane points (midPoints)
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
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        if(isBlocked and not isBlockedByObject) {
            marker.color.a = 1;
            marker.color.r = 1;
            marker.color.g = 0;
            marker.color.b = 0;
        }
        else if(isBlocked and isBlockedByObject){
            marker.color.a = 1;
            marker.color.r = 128.0/255.0;
            marker.color.g = 64.0/255.0;
            marker.color.b = 128.0/255.0;
        }
        else{
            marker.color.a = 1;
            marker.color.r = 0;
            marker.color.g = 1;
            marker.color.b = 0;
        }
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

/**
 * @brief get interpolated smooth lane point when time is t
 * @param t
 * @return Vector2d
 */
Vector2d SmoothLane::evalX(const std::vector<Vector2d, aligned_allocator<Vector2d>>& points_vector, double t){
    int i = 0;
    while(ts[i] < t && i < ts.size()){
        i++;
    }

    if(i == 0){
        return points_vector[0];
    }
    else if(i == ts.size()){
        return points_vector[ts.size()-1];
    }
    else{
        double alpha = (t - ts[i-1]) / (ts[i] - ts[i-1]);
        return alpha * points_vector[i] + (1 - alpha) * points_vector[i-1];
    }
}

/**
 * @brief get interpolated smooth lane point when time is t
 * @param t
 * @return Vector2d
 */
Vector2d SmoothLane::evalX(double t){
    int i = 0;
    while(ts[i] < t && i < ts.size()){
        i++;
    }

    Vector2d vector;
    if(i == 0){
        vector = points[0];
    }
    else if(i == ts.size()){
        vector = points[ts.size()-1];
    }
    else{
        double alpha = (t - ts[i-1]) / (ts[i] - ts[i-1]);
        vector = alpha * points[i] + (1 - alpha) * points[i-1];
    }
    return vector;
}

/**
 * @brief get interpolated side point when time is t
 * @param t
 * @return Vector2d
 */
Vector2d SmoothLane::evalSidePoint(double t, bool isLeft) {
    int i = 0;
    while(ts[i] < t && i < ts.size()){
        i++;
    }

    if(isLeft) {
        if (i == 0) {
            return leftBoundaryPoints[0];
        } else if (i == ts.size()) {
            return leftBoundaryPoints[ts.size() - 1];
        } else {
            double alpha = (t - ts[i - 1]) / (ts[i] - ts[i - 1]);
            return alpha * leftBoundaryPoints[i] + (1 - alpha) * leftBoundaryPoints[i - 1];
        }
    }
    else{
        if (i == 0) {
            return rightBoundaryPoints[0];
        } else if (i == ts.size()) {
            return rightBoundaryPoints[ts.size() - 1];
        } else {
            double alpha = (t - ts[i - 1]) / (ts[i] - ts[i - 1]);
            return alpha * rightBoundaryPoints[i] + (1 - alpha) * rightBoundaryPoints[i - 1];
        }
    }
}

/**
 * @brief get interpolated smooth lane point when time is t
 * @param t
 * @return Vector2d
 */
double SmoothLane::evalWidth(double t){
    int i = 0;
    while(ts[i] < t && i < ts.size()){
        i++;
    }

    if(i == 0){
        return box_size[0];
    }
    else if(i == ts.size()){
        return box_size[ts.size()-1];
    }
    else{
        double alpha = (t - ts[i-1]) / (ts[i] - ts[i-1]);
        return alpha * box_size[i] + (1 - alpha) * box_size[i-1];
    }
}

/**
 * @brief expand corridor from given point
 * @param point, max_box_size
 * @return Corridor
 */
Corridor PlannerBase::expandCorridor(Vector2d point, Vector2d leftBoundaryPoint, Vector2d rightBoundaryPoint, double max_box_size, double map_resolution){
    Corridor corridor;
    corridor.xl = point.x() - max_box_size/2;
    corridor.yl = point.y() - max_box_size/2;
    corridor.xu = point.x() + max_box_size/2;
    corridor.yu = point.y() + max_box_size/2;
    if(isOccupied(point)){
        ROS_ERROR("[PlannerBase] expandCorridor error, point is occluded by obstacles");
        for(int i_mid = 0; i_mid < laneSmooth.points.size(); i_mid++) {
            if (isOccupied(laneSmooth.points[i_mid])) {
                ROS_ERROR("[PlannerBase] expandCorridor error, midPoint is also occluded by obstacles");
            }
        }
        throw -1;
    }

    double epsilon = 0.001;
    for(double x = point.x() - max_box_size/2; x < point.x() + max_box_size/2 + epsilon; x += map_resolution){
        for(double y = point.y() - max_box_size/2; y < point.y() + max_box_size/2 + epsilon; y += map_resolution){
            Vector2d check_point(x, y);
            if(check_point.x() > corridor.xl - epsilon && check_point.x() < corridor.xu + epsilon
               && check_point.y() > corridor.yl - epsilon && check_point.y() < corridor.yu + epsilon
               && isOccupied(check_point))
            {
                float angle = atan2(check_point.y() - point.y(), check_point.x() - point.x());
                if(angle > - M_PI/4 - epsilon && angle < M_PI/4 + epsilon){
                    corridor.xu = check_point.x() - map_resolution;
                }
                else if(angle > 3*M_PI/4 - epsilon || angle < -3*M_PI/4 + epsilon){
                    corridor.xl = check_point.x() + map_resolution;
                }

                if(angle > M_PI/4 - epsilon && angle < 3*M_PI/4 + epsilon){
                    corridor.yu = check_point.y() - map_resolution;
                }
                else if(angle > -3*M_PI/4 - epsilon && angle < - M_PI/4 + epsilon){
                    corridor.yl = check_point.y() + map_resolution;
                }
            }
        }
    }


    //TODO: consider all boundary points!!
    epsilon = 0.001;
    double angle_margin = M_PI/8;
    if(leftBoundaryPoint.x() > corridor.xl + epsilon && leftBoundaryPoint.x() < corridor.xu - epsilon
       && leftBoundaryPoint.y() > corridor.yl + epsilon && leftBoundaryPoint.y() < corridor.yu - epsilon)
    {
        float angle = atan2(leftBoundaryPoint.y() - point.y(), leftBoundaryPoint.x() - point.x());
        if(angle > - M_PI/4 - angle_margin - epsilon && angle < M_PI/4 + angle_margin+ epsilon){
            corridor.xu = leftBoundaryPoint.x();
        }
        else if(angle > 3*M_PI/4 - angle_margin - epsilon || angle < -3*M_PI/4 + angle_margin + epsilon){
            corridor.xl = leftBoundaryPoint.x();
        }

        if(angle > M_PI/4 - angle_margin - epsilon && angle < 3*M_PI/4 + angle_margin + epsilon){
            corridor.yu = leftBoundaryPoint.y();
        }
        else if(angle > -3*M_PI/4 - angle_margin - epsilon && angle < - M_PI/4 + angle_margin + epsilon){
            corridor.yl = leftBoundaryPoint.y();
        }
    }

    if(rightBoundaryPoint.x() > corridor.xl + epsilon && rightBoundaryPoint.x() < corridor.xu - epsilon
       && rightBoundaryPoint.y() > corridor.yl + epsilon && rightBoundaryPoint.y() < corridor.yu - epsilon)
    {
        float angle = atan2(rightBoundaryPoint.y() - point.y(), rightBoundaryPoint.x() - point.x());
        if(angle > - M_PI/4 - angle_margin - epsilon && angle < M_PI/4 + angle_margin+ epsilon){
            corridor.xu = rightBoundaryPoint.x();
        }
        else if(angle > 3*M_PI/4 - angle_margin - epsilon || angle < -3*M_PI/4 + angle_margin + epsilon){
            corridor.xl = rightBoundaryPoint.x();
        }

        if(angle > M_PI/4 - angle_margin - epsilon && angle < 3*M_PI/4 + angle_margin + epsilon){
            corridor.yu = rightBoundaryPoint.y();
        }
        else if(angle > -3*M_PI/4 - angle_margin - epsilon && angle < - M_PI/4 + angle_margin + epsilon){
            corridor.yl = rightBoundaryPoint.y();
        }
    }

    return corridor;
}

/**
 * @brief given ts, construct corridors
 * @param ts, max_box_size, map_resolution
 * @return std::vector<Corridor>
 */
std::vector<Corridor> PlannerBase::expandCorridors(std::vector<double> ts, double map_resolution){
//    for(int i_mid = 0; i_mid < laneSmooth.points.size(); i_mid++) {
//        if (isOccupied(laneSmooth.points[i_mid])) {
//            ROS_ERROR("midpoint error2");
//            throw -1;
//        }
//    }

    std::vector<Corridor> corridors;
    corridors.resize(ts.size());
    for(int i = 0; i < ts.size(); i++){
        Vector2d corridor_point = laneSmooth.evalX(ts[i]);
        Vector2d leftBoundaryPoint = laneSmooth.evalSidePoint(ts[i], true);
        Vector2d rightBoundaryPoint = laneSmooth.evalSidePoint(ts[i], false);
        if(isOccupied(corridor_point)){
            //ROS_WARN("[PlannerBase] heuristic method is used to avoid midpoint collision!");
            double heuristic_margin = 0.1;
            if((corridor_point - leftBoundaryPoint).norm() < (corridor_point - rightBoundaryPoint).norm()){
                while(isOccupied(corridor_point)) {
                    corridor_point =
                            corridor_point + heuristic_margin * (rightBoundaryPoint - leftBoundaryPoint).normalized();
                }
            }
            else{
                while(isOccupied(corridor_point)) {
                    corridor_point =
                            corridor_point + heuristic_margin * (leftBoundaryPoint - rightBoundaryPoint).normalized();
                }
            }
        }
//        ROS_WARN("[PlannerBase] while loop terminated! ");

        Corridor corridor = expandCorridor(corridor_point, leftBoundaryPoint, rightBoundaryPoint, laneSmooth.evalWidth(ts[i]), map_resolution);
        corridors[i] = corridor;
    }
    return corridors;
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
driving_msgs::VehicleCmd PlannerBase::getCurInput(double t){
    double curr_weight = weight_smooth;
//    static int flag = 0;
//    double curr_weight = weight_smooth;
//    static int count_iter =0;
//    cout << "weight: " << curr_weight << endl;
    driving_msgs::VehicleCmd cmd;
    if(!isUseMovingAverage)
    {
        if (flag == 0)
        {
            cmd.steer_angle_cmd =  mpc_result.evalU(t).delta;
            cmd.accel_decel_cmd = mpc_result.evalU(t).alpha;
            ctrl_previous.steer_angle_cmd = mpc_result.evalU(t).delta;
            ctrl_previous.accel_decel_cmd = mpc_result.evalU(t).alpha;
            flag++;
            return cmd;
        }
        else
        {
            cmd.steer_angle_cmd = curr_weight*mpc_result.evalU(t).delta +(1-curr_weight)*ctrl_previous.steer_angle_cmd;
            cmd.accel_decel_cmd = curr_weight*mpc_result.evalU(t).alpha +(1-curr_weight)*ctrl_previous.accel_decel_cmd;
            ctrl_previous.steer_angle_cmd = cmd.steer_angle_cmd;
            ctrl_previous.accel_decel_cmd = cmd.accel_decel_cmd;
            return cmd;
        }
    }
    else
    {
        if(flag<smooth_horizon)
        {
            if(flag == 0)
            {
                cmd.steer_angle_cmd =  mpc_result.evalU(t).delta;
                cmd.accel_decel_cmd = mpc_result.evalU(t).alpha;
                ctrl_history.push_back(cmd);
                flag++;
                return cmd;
            }
            else
            {
                cmd.steer_angle_cmd = curr_weight*mpc_result.evalU(t).delta +(1-curr_weight)*ctrl_previous.steer_angle_cmd;
                cmd.accel_decel_cmd = curr_weight*mpc_result.evalU(t).alpha +(1-curr_weight)*ctrl_previous.accel_decel_cmd;
                ctrl_history.push_back(cmd);
                flag++;
                return cmd;
            }
//
//            cmd.steer_angle_cmd = mpc_result.evalU(t).delta;
//            cmd.accel_decel_cmd = mpc_result.evalU(t).alpha;
//            ctrl_history.push_back(cmd);
//	    flag++;
//            return cmd;
        }
        else
        {
            //cout<<"flag plot"<<flag<<endl;
            ctrl_history[count_iter].steer_angle_cmd = mpc_result.evalU(t).delta;
            ctrl_history[count_iter].accel_decel_cmd = mpc_result.evalU(t).alpha;
            cmd.steer_angle_cmd = 0.0;
            cmd.accel_decel_cmd = 0.0;
            for(int i = 0; i<smooth_horizon;i++)
            {
                cmd.steer_angle_cmd += ctrl_history[i].steer_angle_cmd;
                cmd.accel_decel_cmd += ctrl_history[i].accel_decel_cmd;
            }
            cmd.steer_angle_cmd = cmd.steer_angle_cmd/double(smooth_horizon);
            cmd.accel_decel_cmd = cmd.accel_decel_cmd/double(smooth_horizon);
            count_iter++;
            if(count_iter == smooth_horizon)
            {
                count_iter = 0;
            }
            //cout<<"count_iter is "<< count_iter << endl;
            //cout<<"smooth_horizon plot::"<<smooth_horizon<<endl;
            return cmd;
//            cmd.steer_angle_cmd = 1/double(smooth_horizon)*(mpc_result.evalU(t).delta-ctrl_history.front().steer_angle_cmd) + ctrl_previous.steer_angle_cmd;
//            cmd.accel_decel_cmd = 1/double(smooth_horizon)*(mpc_result.evalU(t).alpha-ctrl_history.front().accel_decel_cmd) + ctrl_previous.accel_decel_cmd;
//            ctrl_previous.steer_angle_cmd = cmd.steer_angle_cmd;
//            ctrl_previous.accel_decel_cmd = cmd.accel_decel_cmd;
//            ctrl_history.push_back(cmd);
//            ctrl_history.pop();
//            return cmd;
        }


    }


};

/**
 * @breif update obstaclePathArray so that it is prediction from [tPredictionStart,tPredictionStart+ horizon]
 * @param tSeq_
 * @param rNominal
 */
void PlannerBase::uploadPrediction(VectorXd tSeq_, double rNominal) {

    VectorXf tSeq = tSeq_.cast<float>();
//    printf("[DEBUG_JBS] obstpath clearing start\n");
    ObstaclePathArray obstaclePathArrayBuffer;

//        obstaclePathArray.obstPathArray.clear();

        for (auto idPredictor : indexedPredictorSet) {
            auto predictor = get<1>(idPredictor);
            //predictor.update_predict(); // this will do nothing if observation is not enough
            if (predictor.is_prediction_available()) {
                vector<geometry_msgs::Pose> obstFuturePose = predictor.eval_pose_seq(tSeq);
                ObstaclePath obstPath;
                for (auto obstPose : obstFuturePose) {
                    // construct one obstaclePath
                    ObstacleEllipse obstE;
                    obstE.q = Vector2d(obstPose.position.x, obstPose.position.y);
                    SE3 poseSE3 = SE3::Identity();
                    Quaterniond q;
                    q.x() = obstPose.orientation.x;
                    q.y() = obstPose.orientation.y;
                    q.z() = obstPose.orientation.z;
                    q.w() = obstPose.orientation.w;

                    poseSE3.rotate(q);

                    Vector3d e1 = poseSE3.rotation().col(0);
                    double theta = atan2(e1(1), e1(0));
                    obstE.theta = theta;

                    obstE.Q.setZero();
                    Matrix2d R;
                    R << cos(theta), -sin(theta), sin(theta), cos(theta);
                    double r1, r2;
                    if (rNominal == 0) {
                        r1 = predictor.getLastDimensions()(0) / sqrt(2);
                        r2 = predictor.getLastDimensions()(1) / sqrt(2);
                    } else {
                        r1 = rNominal;
                        r2 = rNominal;
                    }
                    obstE.r1 = r1;
                    obstE.r2 = r2;
                    obstE.Q(0, 0) = 1 / pow(r1, 2);
                    obstE.Q(1, 1) = 1 / pow(r2, 2);
                    obstE.Q = R * obstE.Q * R.transpose();
                    obstPath.obstPath.push_back(obstE);
                }
                obstaclePathArrayBuffer.obstPathArray.push_back(obstPath);
            }
        }

//    if (mSet[0].try_lock()  ) {
        obstaclePathArray = obstaclePathArrayBuffer;
//        mSet[0].unlock();
//    }

//    printf("[DEBUG_JBS] obstpath pushing finished \n");


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
    if (this->isLPsolved){
        string file_name = log_file_name_base + "_mpc.txt";
        ofstream outfile;
        outfile.open(file_name,std::ios_base::app);
        outfile<< mpc_result.getPretty(t_cur) << endl;
    }
}

/**
 * @brief Get occpuancy
 * @param queryPoint frame = SNU
 * @param isObject  insert true if it is within the obstacle path array
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

/**
 * @brief Verify whether queryPoint is object
 * @param queryPoint frame = SNU
 * @return true if queryPoint is within the obstacle path array
 */
bool PlannerBase::isObject(const Vector2d& queryPoint){
    unsigned long nInspection = 3;

//    printf("[DEBUG_JBS] planner base querying start (number of obst = %d ) \n",obstaclePathArray.obstPathArray.size());

//    mSet[0].lock();
        for (auto obstPath : obstaclePathArray.obstPathArray) {
            for (int i = 0; i < min(nInspection, obstPath.obstPath.size()); i++) {
                ObstacleEllipse obst = obstPath.obstPath[i];
                if (((obst.q - queryPoint).transpose() * obst.Q * (obst.q - queryPoint))(0) < 1) {
//                    ROS_WARN("Query point collided with object");


//                printf("[DEBUG_JBS] planner base querying end \n");

//                    mSet[0].unlock();
                    return true;
                }
            }
        }

//    printf("[DEBUG_JBS] planner base querying end \n");

//    mSet[0].unlock();

        return false;
}

/**
 * @brief Get occpuancy of the line segment (queryPoint1 to queryPoint2)
 * @param queryPoint1 frame = SNU
 * @param queryPoint2 frame = SNU
 * @param isObject  insert true if it is within the obstacle path array
 * @return true if occupied
 */
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

//        // added by JBS to check object collision
//        geometry_msgs::Point p = occupancy_grid_utils::cellCenter(localMap.info,cell);
//        Vector2d queryPoint(p.x,p.y);
//        unsigned long nInspection = 3;
//        for (auto obstPath : obstaclePathArray.obstPathArray){
//            for (int i = 0 ; i < min(nInspection,obstPath.obstPath.size()) ; i ++ ){
//                ObstacleEllipse obst = obstPath.obstPath[i];
//                if (((obst.q - queryPoint).transpose()*obst.Q*(obst.q - queryPoint))(0) < 1 ) {
//                    isObject = true;
//                    ROS_WARN("Query point collided with object");
//                    break;
//                }
//            }
//            if (isObject)
//                break;
//        }

        occupancy_grid_utils::index_t idx = occupancy_grid_utils::cellIndex(localMap.info, cell);
        if(localMap.data[idx] > OCCUPANCY){
            return true;
        }
        iter++;
    }
    return false;
}
