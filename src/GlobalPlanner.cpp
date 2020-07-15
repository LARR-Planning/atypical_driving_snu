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
    // Generate LaneTree
    laneTree.clear();
    Vector2d currentPoint(p_base->cur_state.x, p_base->cur_state.y);
    double lane_length, lane_angle, current_length, alpha;
    Vector2d delta, left_point, mid_point, right_point;
    int i_grid = 0;
    for(int i_lane = 0; i_lane < p_base->laneSliced.points.size() - 1; i_lane++){
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
            bool isNearObject = false;
            for(int k_side = 0; k_side < laneGridPoints.size(); k_side++) {
                if(p_base->isObject(laneGridPoints[k_side])){
                    isNearObject = true;
                    break;
                }
            }

            int start_idx = -1;
            for(int k_side = 0; k_side < laneGridPoints.size(); k_side++){
                if(not p_base->isOccupied(laneGridPoints[k_side])){
                    if(start_idx == -1){
                        start_idx = k_side; //not occupied, start idx not initialized -> initialize start index
                    }
                    if(k_side == laneGridPoints.size() - 1){ // not occupied, laneGrid ended -> add element to tree
                        mid_point = (laneGridPoints[k_side] + laneGridPoints[start_idx])/2;
                        LaneTreeElement laneTreeElement;
                        laneTreeElement.id = i_grid;
                        laneTreeElement.leftBoundaryPoint = laneGridPoints[laneGridPoints.size()-1];
                        laneTreeElement.leftPoint = laneGridPoints[k_side];
                        laneTreeElement.midPoint = mid_point;
                        laneTreeElement.lanePoint = laneGridPoints[(grid_size-1)/2];
                        laneTreeElement.rightPoint = laneGridPoints[start_idx];
                        laneTreeElement.rightBoundaryPoint = laneGridPoints[0];
                        laneTreeElement.width = (laneGridPoints[k_side] - laneGridPoints[start_idx]).norm();
                        laneTreeElement.isNearObject = isNearObject;
                        laneTree.emplace_back(laneTreeElement);
                    }
                }
                else if(start_idx > -1){ //occupied, start idx initialized -> add element to tree
                    mid_point = (laneGridPoints[k_side-1] + laneGridPoints[start_idx])/2;
                    LaneTreeElement laneTreeElement;
                    laneTreeElement.id = i_grid;
                    laneTreeElement.leftBoundaryPoint = laneGridPoints[laneGridPoints.size()-1];
                    laneTreeElement.leftPoint = laneGridPoints[k_side-1];
                    laneTreeElement.midPoint = mid_point;
                    laneTreeElement.lanePoint = laneGridPoints[(grid_size-1)/2];
                    laneTreeElement.rightPoint = laneGridPoints[start_idx];
                    laneTreeElement.rightBoundaryPoint = laneGridPoints[0];
                    laneTreeElement.width = (laneGridPoints[k_side-1] - laneGridPoints[start_idx]).norm();
                    laneTreeElement.isNearObject = isNearObject;
                    laneTree.emplace_back(laneTreeElement);
                    start_idx = -1;
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
    bool isInObject = false;
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
    bool isBlocked = isLaneTreeBlocked(tail.back());
    if(isBlocked){
        tail = cutTail(tail);
    }
    else{
        ROS_WARN_STREAM("Not blocked, obstaclePathArray size:" << p_base->obstaclePathArray.obstPathArray.size());
        for(int i = 0; i < p_base->obstaclePathArray.obstPathArray.size(); i++){
            ROS_WARN_STREAM("Not blocked, obstacle position: (" << p_base->obstaclePathArray.obstPathArray[i].obstPath[0].q(0) << ", " << p_base->obstaclePathArray.obstPathArray[i].obstPath[0].q(1) << ")");
        }
    }

    std::vector<Vector2d, aligned_allocator<Vector2d>> midPoints, lanePoints, leftPoints, rightPoints, leftBoundaryPoints, rightBoundaryPoints;
    midPoints.resize(tail.size());
    lanePoints.resize(tail.size());
    leftPoints.resize(tail.size());
    rightPoints.resize(tail.size());
    leftBoundaryPoints.resize(tail.size());
    rightBoundaryPoints.resize(tail.size());

    for(int i_tail = 0; i_tail < tail.size(); i_tail++){
        midPoints[i_tail] = laneTree[tail[i_tail]].midPoint;
        lanePoints[i_tail] = laneTree[tail[i_tail]].lanePoint;
        leftPoints[i_tail] = laneTree[tail[i_tail]].leftPoint;
        rightPoints[i_tail] = laneTree[tail[i_tail]].rightPoint;
        leftBoundaryPoints[i_tail] = laneTree[tail[i_tail]].leftBoundaryPoint;
        rightBoundaryPoints[i_tail] = laneTree[tail[i_tail]].rightBoundaryPoint;
    }


    ///////////////
    // Smoothing //
    ///////////////
    int idx_start, idx_end, idx_delta, i_smooth;
    Vector2d smoothingPoint;

//    //Smoothing using moving average - first point isolation problem
//    int max_window_size = 30; //TODO: parameterization
//    for(int i_smooth = 1; i_smooth < midPoints.size(); i_smooth++){
//        int idx_window_min = std::max(i_smooth - max_window_size/2, 0);
//        int idx_window_max = std::min(i_smooth + max_window_size/2, (int)midPoints.size() - 1);
//        int window_size = idx_window_max - idx_window_min + 1;
//        Vector2d smoothingPoint(0, 0);
//        for(int i_window = idx_window_min; i_window <= idx_window_max; i_window++){
//            smoothingPoint = smoothingPoint + midPoints[i_window];
//        }
//        smoothingPoint = smoothingPoint/window_size;
//        midPoints[i_smooth] = smoothingPoint;
//    }

    // line smoothing except first point
    i_smooth = 1;
    int window = 20; //TODO: parameterization
    while(i_smooth < midPoints.size()) {
        double bias = (midPoints[i_smooth] - lanePoints[i_smooth]).norm();
        if (bias > 1.0) {  //TODO: parameterization
            //Forward search
            int i_forward = i_smooth + 1;
            while (i_forward < std::min(i_smooth + window, (int) midPoints.size() - 1)) {
                if ((midPoints[i_forward] - lanePoints[i_forward]).norm() < bias / 2) { //TODO: parameterization
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
                    smoothingPoint = (1 - alpha) * midPoints[idx_start] + alpha * midPoints[idx_end];
                    midPoints[idx_start + j_smooth] = smoothingPoint;
                }
            }

            //backward search
            int i_backward = i_smooth - 1;
            while (i_backward > std::max(i_smooth - window, 0)) {
                if ((midPoints[i_backward] - lanePoints[i_backward]).norm() < bias / 2) { //TODO: parameterization
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
                    smoothingPoint = (1 - alpha) * midPoints[idx_start] + alpha * midPoints[idx_end];
                    midPoints[idx_start + j_smooth] = smoothingPoint;
                }
            }
        }
        i_smooth++;
    }

    // lane smoothing at first point
    i_smooth = 0;
    //Forward search
    int i_forward = std::min(i_smooth + window, (int) midPoints.size() - 1);
    if (i_forward - i_smooth > 1) {
        idx_start = i_smooth;
        idx_end = i_forward;
        idx_delta = idx_end - idx_start;

        for (int j_smooth = 1; j_smooth < idx_delta; j_smooth++) {
            alpha = static_cast<double>(j_smooth) / static_cast<double>(idx_delta);
            smoothingPoint = (1 - alpha) * midPoints[idx_start] + alpha * midPoints[idx_end];
            midPoints[idx_start + j_smooth] = smoothingPoint;
        }
    }

    // If midPoints are out of bounds, then push them into bounds
    for (int i_mid = 0; i_mid < midPoints.size(); i_mid++) {
        if (p_base->isOccupied(midPoints[i_mid])) {
            if ((midPoints[i_mid] - leftPoints[i_mid]).norm() <
                (midPoints[i_mid] - rightPoints[i_mid]).norm()) {
                midPoints[i_mid] = leftPoints[i_mid];
            } else {
                midPoints[i_mid] = rightPoints[i_mid];
            }
        }
    }

    // Smoothing using max steering angle - original method
    // Find midAngles
    std::vector<double> midAngles;
    midAngles.resize((int)midPoints.size()-1);
    double angle;
    for(int i_angle = 0; i_angle < midAngles.size(); i_angle++) {
        delta = midPoints[i_angle+1] - midPoints[i_angle];
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
            idx_end = min(i_smooth+2, (int)midPoints.size());
            idx_delta = idx_end - idx_start;

            for(int j_smooth = 1; j_smooth < idx_delta; j_smooth++) {
                alpha = static_cast<double>(j_smooth)/static_cast<double>(idx_delta);
                smoothingPoint = (1-alpha) * midPoints[idx_start] + alpha * midPoints[idx_end];
                midPoints[idx_start+j_smooth] = smoothingPoint;
            }
            for(int j_smooth = 0; j_smooth < idx_delta; j_smooth++) {
                delta = midPoints[idx_start+j_smooth+1] - midPoints[idx_start+j_smooth];
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
    for(int i_mid = 0; i_mid < midPoints.size(); i_mid++) {
        if (p_base->isOccupied(midPoints[i_mid])) {
            if((midPoints[i_mid] - leftPoints[i_mid]).norm() < (midPoints[i_mid] - rightPoints[i_mid]).norm()){
                midPoints[i_mid] = leftPoints[i_mid];
            }
            else{
                midPoints[i_mid] = rightPoints[i_mid];
            }
        }
    }

//    // Smoothing without max steering angle
//    for(int iter_smooth = 0; iter_smooth < 1; iter_smooth++) { //TODO: parameterization
//        i_smooth = 0;
//        while (i_smooth < (int) midPoints.size() - 2) {
//            idx_start = i_smooth;
//            idx_end = i_smooth + 2;
//            idx_delta = idx_end - idx_start;
//
//            for (int j_smooth = 1; j_smooth < idx_delta; j_smooth++) {
//                alpha = static_cast<double>(j_smooth) / static_cast<double>(idx_delta);
//                smoothingPoint = (1 - alpha) * midPoints[idx_start] + alpha * midPoints[idx_end];
//                midPoints[idx_start + j_smooth] = smoothingPoint;
//            }
//            i_smooth++;
//        }
//        // If midPoints are out of bounds, then push them into bounds
//        for (int i_mid = 0; i_mid < midPoints.size(); i_mid++) {
//            if (p_base->isOccupied(midPoints[i_mid], isInObject)) {
//                if ((midPoints[i_mid] - leftPoints[i_mid]).norm() < (midPoints[i_mid] - rightPoints[i_mid]).norm()) {
//                    midPoints[i_mid] = leftPoints[i_mid];
//                } else {
//                    midPoints[i_mid] = rightPoints[i_mid];
//                }
//            }
//        }
//    }



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

    // width allocation
    std::vector<double> box_size;
    box_size.resize(midPoints.size());
    for(int i_mid = 0; i_mid < midPoints.size(); i_mid++) {
        box_size[i_mid] = 2 * std::min((midPoints[i_mid] - leftBoundaryPoints[i_mid]).norm(), (midPoints[i_mid] - rightBoundaryPoints[i_mid]).norm());
    }

    // Time allocation
    double nominal_speed = p_base->laneSpeed;
    std::vector<double> ts;
    ts.resize(midPoints.size());
    ts[0] = t + (midPoints[0] - currentPoint).norm() / nominal_speed;
    for(int i_mid = 1; i_mid < midPoints.size(); i_mid++){
        ts[i_mid] = ts[i_mid-1] + (midPoints[i_mid] - midPoints[i_mid-1]).norm() / nominal_speed;
    }

    SmoothLane smoothLane;
    smoothLane.points = midPoints;
    smoothLane.ts = ts;
    smoothLane.box_size = box_size;
    smoothLane.leftBoundaryPoints = leftBoundaryPoints;
    smoothLane.rightBoundaryPoints = rightBoundaryPoints;
    smoothLane.isBlocked = isBlocked;

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
        double child_width = laneTree[i_tree].width;
        double corridor_width_min;
        if(laneTree[i_tree].isNearObject) { //if child is near object, change corridor_width_min
            corridor_width_min = param.corridor_width_dynamic_min;
        }
        else{
            corridor_width_min = param.corridor_width_min;
        }
        if(!p_base->isOccupied(child_point, current_point) and child_width >= corridor_width_min){
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

bool GlobalPlanner::isLaneTreeBlocked(int last_element_index){
    return laneTree[last_element_index].id != laneTree[laneTree.size()-1].id;
}

std::vector<int> GlobalPlanner::cutTail(const std::vector<int>& tail) const{
    int tail_end = (int)tail.size() - 1 - static_cast<int>((param.safe_distance + SP_EPSILON)/param.grid_resolution);
    if(tail_end < 1){
        tail_end = 1;
    }

    return vector<int>(tail.begin(), tail.begin() + tail_end);
}