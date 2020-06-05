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

