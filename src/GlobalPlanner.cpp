#include <atypical_planner/GlobalPlanner.h>
#include <third_party/jps.h>
#include <octomap_msgs/conversions.h>

#define SP_EPSILON          1e-9
#define SP_EPSILON_FLOAT    1e-4
#define SP_INFINITY         1e+9
#define PI                  3.1415

using namespace Planner;

GlobalPlanner::GlobalPlanner(const Planner::ParamGlobal &g_param,
                             shared_ptr<PlannerBase> p_base_) : AbstractPlanner(p_base_),param(g_param) {
    printf("[GlobalPlanner] Init.\n");

}

/**
 * @brief moniter whether current path is feasible against the octomap
 * @return
 */
bool GlobalPlanner::isCurTrajFeasible() {
    return isFeasible;
}

/**
 * @brief Global planning routine
 * @return true if success
 */
bool GlobalPlanner::plan(double t) {
    if(p_base->localMap.data.empty()){
        ROS_WARN("[GlobalPlanner] localmap.data is empty");
        return false;
    }

    // Generate LaneTree
    laneTree.clear();
    laneTreePath.clear();
    Vector2d currentPoint(p_base->cur_state.x, p_base->cur_state.y);
    double lane_length, lane_angle, current_length, alpha;
    Vector2d delta, left_point, mid_point, right_point;
    int i_grid = 0;
    for(int i_lane = 0; i_lane < (int)p_base->laneSliced.points.size() - 1; i_lane++){
        int grid_size = 2 * floor(p_base->laneSliced.widths[i_lane]/2/param.grid_resolution) + 1;
        delta = p_base->laneSliced.points[i_lane+1] - p_base->laneSliced.points[i_lane];
        lane_length = delta.norm();
        lane_angle = atan2(delta.y(), delta.x());
        current_length = 0;
        while(current_length < lane_length){
            // slice laneSliced to generate laneGridPoints and find side points
            vector<Vector2d> laneGridPoints;
            laneGridPoints.resize(grid_size);
            alpha = current_length / lane_length;
            mid_point = alpha * p_base->laneSliced.points[i_lane+1] + (1-alpha) * p_base->laneSliced.points[i_lane];
            laneGridPoints[(grid_size-1)/2] = mid_point;
            for(int j_side = 1; j_side <= (grid_size-1)/2; j_side++){
                left_point = mid_point + j_side * param.grid_resolution * Vector2d(cos(lane_angle + M_PI/2),sin(lane_angle + M_PI/2));
                right_point = mid_point + j_side * param.grid_resolution * Vector2d(cos(lane_angle - M_PI/2),sin(lane_angle - M_PI/2));
                laneGridPoints[(grid_size-1)/2 + j_side] = left_point;
                laneGridPoints[(grid_size-1)/2 - j_side] = right_point;
            }
            // check occupancy of side points and construct laneTree
            std::vector<int> gridPointStates;
            gridPointStates.resize(laneGridPoints.size());
            for(int k_side = 0; k_side < laneGridPoints.size(); k_side++) {
                double object_velocity = 0;
                if(p_base->isObject(laneGridPoints[k_side], param.max_obstacle_prediction_query_size, object_velocity)
                   and object_velocity > param.object_velocity_threshold){
                    gridPointStates[k_side] = 2;
                }
                else if(p_base->isOccupied(laneGridPoints[k_side])){
                    gridPointStates[k_side] = 1;
                }
                else{
                    gridPointStates[k_side] = 0;
                }
            }

            for(int k_side = 0; k_side < laneGridPoints.size(); k_side++) {
                if(gridPointStates[k_side] == 2) {
                    for(int k_expand = k_side - 1; k_expand > -1; k_expand--){
                        if(gridPointStates[k_expand] > 0) {
                            gridPointStates[k_expand] = 2;
                        }
                        else {
                            break;
                        }
                    }
                    for(int k_expand = k_side + 1; k_expand < laneGridPoints.size(); k_expand++){
                        if(gridPointStates[k_expand] > 0) {
                            gridPointStates[k_expand] = 2;
                        }
                        else {
                            break;
                        }
                    }
                }
            }
            int start_idx = -1;
            for(int k_side = 0; k_side < laneGridPoints.size(); k_side++){
                if(gridPointStates[k_side] == 0){
                    if(start_idx == -1){
                        start_idx = k_side; //not occupied, start idx not initialized -> initialize start index
                    }
                    if(k_side == (int)laneGridPoints.size() - 1){ // not occupied, laneGrid ended -> add element to tree
                        mid_point = (laneGridPoints[k_side] + laneGridPoints[start_idx])/2;
                        LaneTreeElement laneTreeElement;
                        laneTreeElement.id = i_grid;
                        laneTreeElement.leftBoundaryPoint = laneGridPoints[(int)laneGridPoints.size()-1];
                        laneTreeElement.leftPoint = laneGridPoints[k_side];
                        laneTreeElement.midPoint = mid_point;
                        laneTreeElement.lanePoint = laneGridPoints[(grid_size-1)/2];
                        laneTreeElement.rightPoint = laneGridPoints[start_idx];
                        laneTreeElement.rightBoundaryPoint = laneGridPoints[0];
                        laneTreeElement.width = (laneGridPoints[k_side] - laneGridPoints[start_idx]).norm();
                        if(start_idx > 0){
                            laneTreeElement.isNearObject = (gridPointStates[start_idx - 1] == 2);
                        }
                        else{
                            laneTreeElement.isNearObject = false;
                        }
                        laneTree.emplace_back(laneTreeElement);
//                        if(debug) {
//                            ROS_WARN_STREAM("segment at (" << laneTreeElement.leftPoint.x() << ","
//                                                           << laneTreeElement.leftPoint.y() << ") to ("
//                                                           << laneTreeElement.rightPoint.x() << ","
//                                                           << laneTreeElement.rightPoint.y() << ")");
//                            if(laneTreeElement.isNearObject){
//                                ROS_WARN_STREAM("Near object");
//                            }
//                            else{
//                                ROS_WARN_STREAM("Not near object");
//                            }
//                        }
                    }
                }
                else if(start_idx > -1){ //occupied, start idx initialized -> add element to tree
                    mid_point = (laneGridPoints[k_side-1] + laneGridPoints[start_idx])/2;
                    LaneTreeElement laneTreeElement;
                    laneTreeElement.id = i_grid;
                    laneTreeElement.leftBoundaryPoint = laneGridPoints[(int)laneGridPoints.size()-1];
                    laneTreeElement.leftPoint = laneGridPoints[k_side-1];
                    laneTreeElement.midPoint = mid_point;
                    laneTreeElement.lanePoint = laneGridPoints[(grid_size-1)/2];
                    laneTreeElement.rightPoint = laneGridPoints[start_idx];
                    laneTreeElement.rightBoundaryPoint = laneGridPoints[0];
                    laneTreeElement.width = (laneGridPoints[k_side-1] - laneGridPoints[start_idx]).norm();
                    if(gridPointStates[k_side] == 2 or (start_idx > 0 and gridPointStates[start_idx - 1] == 2)){
                        laneTreeElement.isNearObject = true;
                    }
                    else{
                        laneTreeElement.isNearObject = false;
                    }
                    laneTree.emplace_back(laneTreeElement);
                    start_idx = -1;
//                    if(debug) {
//                        ROS_WARN_STREAM("segment at (" << laneTreeElement.leftPoint.x() << ","
//                                                       << laneTreeElement.leftPoint.y() << ") to ("
//                                                       << laneTreeElement.rightPoint.x() << ","
//                                                       << laneTreeElement.rightPoint.y() << ")");
//                        if(laneTreeElement.isNearObject){
//                            ROS_WARN_STREAM("Near object");
//                        }
//                        else{
//                            ROS_WARN_STREAM("Not near object");
//                        }
//                    }
                }
                else{ //occupied, start idx not initialized -> do nothing
                    start_idx = -1;
                }
            }
            i_grid++;
            current_length += param.grid_resolution;
        }
    }


    // Allocate children of each node in laneTree
    for(int i_tree = 0; i_tree < laneTree.size(); i_tree++){
        laneTree[i_tree].children = findChildren(i_tree);
    }


    // Find start node of laneTree
    int i_tree_start = -1;
    double dist, dist_start = SP_INFINITY;
    for(int i_tree = 0; i_tree < laneTree.size(); i_tree++) {
        if (laneTree[i_tree].id > 0) {
            break;
        }
        dist = (laneTree[i_tree].midPoint - currentPoint).norm();
        if(not p_base->isOccupied(laneTree[i_tree].midPoint, currentPoint)){
            if (i_tree_start == -1 || dist < dist_start) {
                i_tree_start = i_tree;
                dist_start = dist;
            }
        }
    }
    if(i_tree_start == -1){
        int i_shortest = -1;
        for(int i_tree = 0; i_tree < laneTree.size(); i_tree++) {
            if (laneTree[i_tree].id > 0) {
                break;
            }
            dist = (laneTree[i_tree].midPoint - currentPoint).norm();
            if (dist < dist_start) {
                i_shortest = i_tree;
                dist_start = dist;
            }
        }
        i_tree_start = i_shortest;
    }
    laneTree[i_tree_start].midPoint = currentPoint; // Fix start node to reflect currentState


    // Update laneTree to find midPoints
    laneTreeSearch(i_tree_start);
    std::vector<int> tail = getMidPointsFromLaneTree(i_tree_start);
    laneTreePath.resize(tail.size());
    for(int i_tail = 0; i_tail < tail.size(); i_tail++){
        laneTreePath[i_tail] = laneTree[tail[i_tail]];
    }


    // Smoothing
    int idx_start, idx_end, idx_delta, i_smooth, window;
    double window_length;
    Vector2d smoothingPoint;

    // line smoothing except first point
    i_smooth = 1;
    while(i_smooth < laneTreePath.size()) {
        double bias = (laneTreePath[i_smooth].midPoint - laneTreePath[i_smooth].lanePoint).norm();
        if (bias > param.smoothing_cliff_min_bias) {
            //Forward search
            int i_forward = i_smooth + 1;
            window = 0;
            window_length = 0;
            while(window < (int) laneTreePath.size() - i_smooth - 1 and window_length < param.smoothing_distance){
                window_length += (laneTreePath[i_smooth + window + 1].lanePoint - laneTreePath[i_smooth + window].lanePoint).norm();
                window++;
            }

            while (i_forward < std::min(i_smooth + window, (int) laneTreePath.size() - 1)) {
                if ((laneTreePath[i_forward].midPoint - laneTreePath[i_forward].lanePoint).norm() < bias * param.smoothing_cliff_ratio) {
                    i_forward++;
                } else {
                    i_forward--;
                    break;
                }
            }
            if (i_forward - i_smooth > 1) {
                idx_start = i_smooth;
                idx_end = i_forward;
                idx_delta = idx_end - idx_start;

                for (int j_smooth = 1; j_smooth < idx_delta; j_smooth++) {
                    alpha = static_cast<double>(j_smooth) / static_cast<double>(idx_delta);
                    smoothingPoint = (1 - alpha) * laneTreePath[idx_start].midPoint + alpha * laneTreePath[idx_end].midPoint;
                    laneTreePath[idx_start + j_smooth].midPoint = smoothingPoint;
                }
            }

            //backward search
            int i_backward = i_smooth - 1;
            window = 0;
            window_length = 0;
            while(window < (int) i_smooth - 1 and window_length < param.smoothing_distance){
                window_length += (laneTreePath[i_smooth - window - 1].lanePoint - laneTreePath[i_smooth - window].lanePoint).norm();
                window++;
            }

            while (i_backward > std::max(i_smooth - window, 0)) {
                if ((laneTreePath[i_backward].midPoint - laneTreePath[i_backward].lanePoint).norm() < bias * param.smoothing_cliff_ratio) {
                    i_backward--;
                } else {
                    i_backward++;
                    break;
                }
            }
            if (i_smooth - i_backward > 1) {
                idx_start = i_backward;
                idx_end = i_smooth;
                idx_delta = idx_end - idx_start;

                for (int j_smooth = 1; j_smooth < idx_delta; j_smooth++) {
                    alpha = static_cast<double>(j_smooth) / static_cast<double>(idx_delta);
                    smoothingPoint = (1 - alpha) * laneTreePath[idx_start].midPoint + alpha * laneTreePath[idx_end].midPoint;
                    laneTreePath[idx_start + j_smooth].midPoint = smoothingPoint;
                }
            }
        }
        i_smooth++;
    }


    // lane smoothing at first point with Bernstein Polynomial
    i_smooth = 0;
    window = 0;
    window_length = 0;
    while(window < (int) laneTreePath.size() - i_smooth - 1 and window_length < param.smoothing_distance){
        window_length += (laneTreePath[i_smooth + window + 1].lanePoint - laneTreePath[i_smooth + window].lanePoint).norm();
        window++;
    }

    int i_forward = std::min(i_smooth + window, (int) laneTreePath.size() - 1);
    int n = 3;
    std::vector<Vector2d, aligned_allocator<Vector2d>> controlPoints;
    controlPoints.resize(4);
    controlPoints[0] = laneTreePath[i_smooth].midPoint;
    controlPoints[1] = laneTreePath[i_smooth].midPoint + Vector2d(cos(p_base->cur_state.theta), sin(p_base->cur_state.theta)) * (p_base->cur_state.v/n);
    controlPoints[2] = laneTreePath[i_forward].midPoint - (p_base->laneSliced.points[1] - p_base->laneSliced.points[0]).normalized() * (p_base->laneSpeed/n);
    controlPoints[3] = laneTreePath[i_forward].midPoint;

    if (i_forward - i_smooth > 1) {
        idx_start = i_smooth;
        idx_end = i_forward;
        idx_delta = idx_end - idx_start;
        for (int j_smooth = 1; j_smooth < idx_delta; j_smooth++) {
            alpha = static_cast<double>(j_smooth) / static_cast<double>(idx_delta);
            smoothingPoint = getPointFromControlPoints(controlPoints, alpha);
            laneTreePath[idx_start + j_smooth].midPoint = smoothingPoint;
        }
    }

    // If midPoints are out of bounds, then push them into bounds
    for (int i_mid = 0; i_mid < laneTreePath.size(); i_mid++) {
        if (p_base->isOccupied(laneTreePath[i_mid].midPoint)) {
            if ((laneTreePath[i_mid].midPoint - laneTreePath[i_mid].leftPoint).norm() <
                (laneTreePath[i_mid].midPoint - laneTreePath[i_mid].rightPoint).norm()) {
                laneTreePath[i_mid].midPoint = laneTreePath[i_mid].leftPoint;
            } else {
                laneTreePath[i_mid].midPoint = laneTreePath[i_mid].rightPoint;
            }
        }
    }


    // Smoothing using max steering angle - original method
    // Find midAngles
    std::vector<double> midAngles;
    midAngles.resize((int)laneTreePath.size()-1);
    double angle;
    for(int i_angle = 0; i_angle < midAngles.size(); i_angle++) {
        delta = laneTreePath[i_angle+1].midPoint - laneTreePath[i_angle].midPoint;
        angle = atan2(delta.y(), delta.x());
        if(angle < 0) {
            angle = angle + 2 * M_PI;
        }
        midAngles[i_angle] = angle;
    }

    i_smooth = 0;
    while(i_smooth < (int)midAngles.size()-2) {
        double diff = abs(midAngles[i_smooth+1] - midAngles[i_smooth]);
        if(diff > M_PI) {
            diff = 2 * M_PI - diff;
        }

        if(diff > param.max_steering_angle) {
            idx_start = i_smooth;
            idx_end = min(i_smooth+2, (int)laneTreePath.size());
            idx_delta = idx_end - idx_start;

            for(int j_smooth = 1; j_smooth < idx_delta; j_smooth++) {
                alpha = static_cast<double>(j_smooth)/static_cast<double>(idx_delta);
                smoothingPoint = (1-alpha) * laneTreePath[idx_start].midPoint + alpha * laneTreePath[idx_end].midPoint;
                laneTreePath[idx_start+j_smooth].midPoint = smoothingPoint;
            }
            for(int j_smooth = 0; j_smooth < idx_delta; j_smooth++) {
                delta = laneTreePath[idx_start+j_smooth+1].midPoint - laneTreePath[idx_start+j_smooth].midPoint;
                angle = atan2(delta.y(), delta.x());
                if(angle < 0){
                    angle = angle + 2 * M_PI;
                }
                midAngles[idx_start+j_smooth] = angle;
            }
            i_smooth = max(i_smooth - 1, 0);
        }
        else{
            i_smooth++;
        }
    }
    // If midPoints are out of bounds, then push them into bounds
    for(int i_mid = 0; i_mid < laneTreePath.size(); i_mid++) {
        if (p_base->isOccupied(laneTreePath[i_mid].midPoint)) {
            if((laneTreePath[i_mid].midPoint - laneTreePath[i_mid].leftPoint).norm() < (laneTreePath[i_mid].midPoint - laneTreePath[i_mid].rightPoint).norm()){
                laneTreePath[i_mid].midPoint = laneTreePath[i_mid].leftPoint;
            }
            else{
                laneTreePath[i_mid].midPoint = laneTreePath[i_mid].rightPoint;
            }
        }
    }


//    // Computing curvature (JBS)
//    double meanCurv = meanCurvature(midPoints);
//    double rho_thres = param.curvature_thres;
//    double vLaneRef;
//    double vmin = param.car_speed_min;
//    double vmax = param.car_speed_max;
//    if (meanCurv > rho_thres)
//        vLaneRef = vmin;
//    else{
//        vLaneRef = vmax - (vmax-vmin)/rho_thres*meanCurv;
//    }
//
//    p_base->mSet[1].lock();
//    p_base->laneSpeed = vLaneRef;
//    p_base->laneCurvature = meanCurv;
//    ROS_INFO("lane [%f,%f]" ,meanCurv,vLaneRef);
//    p_base->mSet[1].unlock();

    // check width and cut tail
    bool isBlocked = false;
    bool isBlockedByObject = false;
    int tail_end = findLaneTreePathTail(isBlocked, isBlockedByObject);
    if(not isBlocked){
        ROS_WARN_STREAM("[GlobalPlanner] Not blocked, obstaclePathArray size:" << p_base->obstaclePathArray.obstPathArray.size());
        for(int i = 0; i < p_base->obstaclePathArray.obstPathArray.size(); i++){
            ROS_WARN_STREAM("[GlobalPlanner] Not blocked, obstacle position: (" << p_base->obstaclePathArray.obstPathArray[i].obstPath[0].q(0) << ", " << p_base->obstaclePathArray.obstPathArray[i].obstPath[0].q(1) << ")");
        }
    }

    // width allocation
    std::vector<double> box_size;
    box_size.resize(tail_end);
    for(int i_mid = 0; i_mid < tail_end; i_mid++) {
        box_size[i_mid] = 2 * std::min((laneTreePath[i_mid].midPoint - laneTreePath[i_mid].leftBoundaryPoint).norm(),
                                       (laneTreePath[i_mid].midPoint - laneTreePath[i_mid].rightBoundaryPoint).norm());
    }


    // Calculate curvature  (JBS)

    vector<Vector2d, aligned_allocator<Vector2d>> pathSliced;

    for (int i_mid = 0; i_mid < tail_end ; i_mid++)
        pathSliced.emplace_back(laneTreePath[i_mid].midPoint);
    double meanCurv = meanCurvature(pathSliced);


    // Nominal speed (JBS);

    double vmin = param.car_speed_min;
    double vmax = param.car_speed_max;
    double rho_thres = param.curvature_thres;
    double vLaneRef; // referance velocity for the current lane
    // ref speed determined
    if (meanCurv > rho_thres)
        vLaneRef = vmin;
    else{
        vLaneRef = vmax - (vmax-vmin)/rho_thres*meanCurv;
    }

    p_base->laneSpeed = (1-param.v_ref_past_weight)*vLaneRef + param.v_ref_past_weight*p_base->laneSpeed; // mixing
    p_base->laneCurvature = meanCurv;
    ROS_INFO("lane [%f,%f]" ,meanCurv,vLaneRef);

    // Time allocation
    double nominal_acceleration = param.nominal_acceleration;
    double nominal_speed = p_base->laneSpeed;
    double total_length = 0;
    for(int i_mid = 1; i_mid < tail_end; i_mid++){
        total_length += (laneTreePath[i_mid].midPoint - laneTreePath[i_mid-1].midPoint).norm();
    }

    std::vector<double> ts;
    ts.resize(tail_end);
    ts[0] = t;

    double alloc_acceleration, delta_length;
    double initial_speed = p_base->cur_state.v;
    double alloc_speed = initial_speed;
    double alloc_length = 0;
    for(int i_mid = 1; i_mid < tail_end; i_mid++){
        delta_length = (laneTreePath[i_mid].midPoint - laneTreePath[i_mid-1].midPoint).norm();
        if(delta_length < SP_EPSILON_FLOAT){
            ts[i_mid] = ts[i_mid - 1];
        }
        else {
            if (total_length - alloc_length < pow(alloc_speed, 2) / (2 * nominal_acceleration)) { // start deceleration
                alloc_acceleration = -alloc_speed * alloc_speed * 0.5 / (total_length - alloc_length);
            } else if (alloc_speed <= nominal_speed) {
                alloc_acceleration = std::min(
                        (nominal_speed * nominal_speed - alloc_speed * alloc_speed) / (2 * delta_length),
                        nominal_acceleration);
            } else {
                alloc_acceleration = std::max(
                        (nominal_speed * nominal_speed - alloc_speed * alloc_speed) / (2 * delta_length),
                        -nominal_acceleration);
            }
            if(abs(alloc_acceleration) < SP_EPSILON_FLOAT){
                ts[i_mid] = ts[i_mid - 1] + delta_length / nominal_speed;
                alloc_speed = nominal_speed;
            } else{
                double new_speed_sq = alloc_speed * alloc_speed + 2 * alloc_acceleration * delta_length;
                if(new_speed_sq < 0){
                    new_speed_sq = 0;
                }
                double new_speed = sqrt(new_speed_sq);
                ts[i_mid] = ts[i_mid - 1] + (new_speed - alloc_speed) / alloc_acceleration;
                alloc_speed = new_speed;
            }
            alloc_length += delta_length;
        }

//        if(isnan(ts[i_mid])){
//            int debug = 0;
//        }
    }
    
    // reformat
    std::vector<Vector2d, aligned_allocator<Vector2d>> midPoints, leftBoundaryPoints, rightBoundaryPoints;
    midPoints.resize(tail_end);
    leftBoundaryPoints.resize(tail_end);
    rightBoundaryPoints.resize(tail_end);
    for(int i_tail = 0; i_tail < tail_end; i_tail++){
        midPoints[i_tail] = laneTreePath[i_tail].midPoint;
        leftBoundaryPoints[i_tail] = laneTreePath[i_tail].leftBoundaryPoint;
        rightBoundaryPoints[i_tail] = laneTreePath[i_tail].rightBoundaryPoint;
    }

    SmoothLane smoothLane;
    smoothLane.points = midPoints;
    smoothLane.ts = ts;
    smoothLane.box_size = box_size;
    smoothLane.leftBoundaryPoints = leftBoundaryPoints;
    smoothLane.rightBoundaryPoints = rightBoundaryPoints;
    smoothLane.isBlocked = isBlocked;
    smoothLane.isBlockedByObject = isBlockedByObject;

    p_base->mSet[1].lock();
    smoothLane.n_total_markers = p_base->laneSmooth.n_total_markers;
    p_base->laneSmooth = smoothLane;
    p_base->mSet[1].unlock();

    return true; // change this line properly
}

bool GlobalPlanner::intersect(Point i0, Point i1, Point j0, Point j1){
    int ab = ccw(i0, i1, j0)*ccw(i0, i1, j1);
    int cd = ccw(j0, j1, i0)*ccw(j0, j1, i1);
    if (ab == 0 && cd == 0) {
//        if (a > b)swap(a, b);
//        if (c > d)swap(c, d);
//        return c <= b && a <= d;
        return false;
    }
    return ab <= 0 && cd <= 0;
}

int GlobalPlanner::ccw(Point a, Point b, Point c) {
    double op = a.x*b.y + b.x*c.y + c.x*a.y;
    op -= (a.y*b.x + b.y*c.x + c.y*a.x);
    if (op > SP_EPSILON) return 1;
    else if (abs(op) < SP_EPSILON) return 0;
    else return -1;
}

/**Grid generation
 * @brief update the global planning result to the shared resource
 */
void GlobalPlanner::updateCorridorToBase() {

    // update routine here
    p_base->setCorridorSeq(curCorridorSeq); // just an example
    p_base->setSearchRange(search_range);
}

std::array<double, 4> GlobalPlanner::boxTransform(const SE3& Tab, const std::array<double, 4>& box){
    std::array<double, 4> box_transformed;
    Vector3d transformed_vector1 = applyTransform(Tab, Vector3d(box[0], box[1], 0)); //transform SNU to world
    Vector3d transformed_vector2 = applyTransform(Tab, Vector3d(box[2], box[3], 0)); //transform SNU to world

    box_transformed[0] = min(transformed_vector1(0), transformed_vector2(0));
    box_transformed[1] = min(transformed_vector1(1), transformed_vector2(1));
    box_transformed[2] = max(transformed_vector1(0), transformed_vector2(0));
    box_transformed[3] = max(transformed_vector1(1), transformed_vector2(1));

    return box_transformed;
}

std::vector<int> GlobalPlanner::findChildren(int idx){
    std::vector<int> children;
    if(laneTree[idx].id == laneTree.back().id){
        return children;
    }

    double shortestDist = SP_INFINITY;
    for(int i_tree = idx + 1; i_tree < laneTree.size(); i_tree++){
        if(laneTree[i_tree].id == laneTree[idx].id){
            continue;
        }
        if(laneTree[i_tree].id != laneTree[idx].id + 1){
            break;
        }

        Vector2d current_point = laneTree[idx].midPoint;
        Vector2d child_point = laneTree[i_tree].midPoint;
        if(!p_base->isOccupied(child_point, current_point)){
            double dist = (child_point - current_point).norm();
            if(children.empty()){
                children.emplace_back(i_tree);
                shortestDist = dist;
            }
            else if(dist < shortestDist){
                children.insert(children.begin(), i_tree);
                shortestDist = dist;
            }
            else{
                children.emplace_back(i_tree);
            }
        }
    }
    return children;
}

void GlobalPlanner::laneTreeSearch(int i_tree){
    // terminal condition
    if(laneTree[i_tree].children.empty()){
        laneTree[i_tree].total_width = laneTree[i_tree].width;
        laneTree[i_tree].visited = true;
        laneTree[i_tree].next = -1; // end of the node
        return;
    }

    // update all children
    for(int i_child = 0; i_child < laneTree[i_tree].children.size(); i_child++){
        if(not laneTree[i_tree].visited) {
            laneTreeSearch(laneTree[i_tree].children[i_child]);
        }
    }

    // find longest path
    for(int i_child = 0; i_child < laneTree[i_tree].children.size(); i_child++){
        int i_tree_child = laneTree[i_tree].children[i_child];
        if(laneTree[i_tree].distance < laneTree[i_tree_child].distance + 1
            || (laneTree[i_tree].distance == laneTree[i_tree_child].distance + 1
                && laneTree[i_tree].total_width < laneTree[i_tree_child].total_width + laneTree[i_tree].width)){
            laneTree[i_tree].distance = laneTree[i_tree_child].distance + 1;
            laneTree[i_tree].total_width = laneTree[i_tree_child].total_width + laneTree[i_tree].width;
            laneTree[i_tree].next = i_tree_child;
        }
    }

    laneTree[i_tree].visited = true;
}

std::vector<int> GlobalPlanner::getMidPointsFromLaneTree(int i_tree_start){
    std::vector<int> tail;
    int i_tree = i_tree_start;
    while(laneTree[i_tree].next != -1){
        tail.emplace_back(i_tree);
        i_tree = laneTree[i_tree].next;
    }
    tail.emplace_back(i_tree);
    return tail;
}

int GlobalPlanner::findLaneTreePathTail(bool& isBlocked, bool& isBlockedByObject){
    isBlocked = false;
    isBlockedByObject = false;
    int tail_end = (int)laneTreePath.size() - 1;
    double corridor_width_min;
    for(int i_tree = 0; i_tree < laneTreePath.size(); i_tree++){
        if(laneTreePath[i_tree].isNearObject) {
            corridor_width_min = param.corridor_width_dynamic_min;
        }
        else{
            corridor_width_min = param.corridor_width_min;
        }

        if(laneTreePath[i_tree].width < corridor_width_min){
            ROS_WARN("[GlobalPlanner] lanePath blocked");
            tail_end = std::max(i_tree - 1, 1);
            isBlocked = true;
            if(laneTreePath[i_tree].isNearObject) {
                ROS_WARN("[GlobalPlanner] lanePath blocked by dynamic obstacle");
                isBlockedByObject = true;
            }
            break;
        }
    }

    //safe distance
    if(isBlocked){
        int window = 0;
        double window_length = 0;
        while(window < tail_end - 1 and window_length < param.safe_distance){
            window_length += (laneTreePath[tail_end - window - 1].lanePoint - laneTreePath[tail_end - window].lanePoint).norm();
            window++;
        }

        tail_end = tail_end - window;
        if(window_length < param.safe_distance){
            tail_end = 1;
        }
    }

    return tail_end;
}

//bool GlobalPlanner::isLaneTreeBlocked(int last_element_index){
//    return laneTree[last_element_index].id != laneTree[laneTree.size()-1].id;
//}
//
//bool GlobalPlanner::isLaneTreeBlockedByDynamicObs(int last_element_index){
//    for(int i_tree = last_element_index + 1; i_tree < laneTree.size(); i_tree++){
//        if(laneTree[i_tree].id == laneTree[last_element_index].id){
//            continue;
//        }
//        else if(laneTree[i_tree].id == laneTree[last_element_index].id + 1){
//            if(laneTree[i_tree].isNearObject){
//                return true;
//            }
//        }
//        else{
//            break;
//        }
//    }
//    return false;
//}

//std::vector<int> GlobalPlanner::cutTail(const std::vector<int>& tail) const{
//    int tail_end = (int)tail.size() - 1 - static_cast<int>((param.safe_distance + SP_EPSILON)/param.grid_resolution);
//    if(tail_end < 1){
//        tail_end = 1;
//    }
//
//    return vector<int>(tail.begin(), tail.begin() + tail_end);
//}

int GlobalPlanner::nChoosek(int n, int k){
    if(k > n) return 0;
    if(k * 2 > n) k = n-k;
    if(k == 0) return 1;

    int result = n;
    for(int i = 2; i <= k; i++){
        result *= (n-i+1);
        result /= i;
    }
    return result;
}

double GlobalPlanner::getBernsteinBasis(int n, int i, double t_normalized){
    return nChoosek(n, i) * pow(t_normalized, i) * pow(1-t_normalized, n - i);
}

Vector2d GlobalPlanner::getPointFromControlPoints(std::vector<Vector2d, aligned_allocator<Vector2d>> control_points, double t_normalized){
    Vector2d point;
    double t = t_normalized;
    if(t < 0 - SP_EPSILON || t > 1 + SP_EPSILON){
        ROS_ERROR("[GlobalPlanner] Input of getPointFromControlPoints is out of bound");
        throw -1;
    }

    int n_ctrl = (int)control_points.size() - 1;
    double x = 0, y = 0;
    for(int i = 0; i < n_ctrl + 1; i++){
        double b_i_n = getBernsteinBasis(n_ctrl, i, t_normalized);
        x += control_points[i].x() * b_i_n;
        y += control_points[i].y() * b_i_n;
    }

    point = Vector2d(x, y);
    return point;
}