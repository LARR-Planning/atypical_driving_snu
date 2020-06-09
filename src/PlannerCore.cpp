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
vector<Vector2d> Lane::slicing(const CarState &curCarState, Vector2d windowOrig, double w, double h) {

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

    // Then, let us select the final index
    int EndPointIdx = StartPointIdx;
    for (int i = StartPointIdx ; i < points.size()  ; i++){
        EndPointIdx = i;
        Vector2d point = points[i-1]; // next point
        bool isInWindow = (point(0) < windowOrig(0) + w) and
                          (point(0) > windowOrig(0) - w) and
                          (point(1) < windowOrig(1) + h) and
                          (point(1) > windowOrig(1) - h);

        if (not isInWindow)
            break;

    }
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


