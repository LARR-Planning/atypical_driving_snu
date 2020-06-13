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
//        int grid_size = 2 * floor(p_base->laneSliced.widths[i_lane]/2/param.grid_resolution) + 1; //TODO: fix to this
        int grid_size = 2 * floor(6/2/param.grid_resolution) + 1;
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
            int start_idx = -1;
            for(int k_side = 0; k_side < laneGridPoints.size(); k_side++){
                if(!p_base->isOccupied(laneGridPoints[k_side])){
                    if(start_idx == -1){
                        start_idx = k_side;
                    }
                    if(k_side == laneGridPoints.size()-1){
                        mid_point = (laneGridPoints[k_side] + laneGridPoints[start_idx])/2;
                        std::vector<int> parents = findParents(i_grid, mid_point);
                        LaneTreeElement laneTreeElement;
                        laneTreeElement.id = i_grid;
                        laneTreeElement.leftPoint = laneGridPoints[k_side];
                        laneTreeElement.midPoint = mid_point;
                        laneTreeElement.rightPoint = laneGridPoints[start_idx];
                        laneTreeElement.parents = parents;
                        laneTree.emplace_back(laneTreeElement);
                    }
                }
                else if(start_idx > -1){
                    mid_point = (laneGridPoints[k_side-1] + laneGridPoints[start_idx])/2;
                    std::vector<int> parents = findParents(i_grid, mid_point);
                    LaneTreeElement laneTreeElement;
                    laneTreeElement.id = i_grid;
                    laneTreeElement.leftPoint = laneGridPoints[k_side-1];
                    laneTreeElement.midPoint = mid_point;
                    laneTreeElement.rightPoint = laneGridPoints[start_idx];
                    laneTreeElement.parents = parents;
                    laneTree.emplace_back(laneTreeElement);
                    start_idx = -1;
                }
                else{
                    start_idx = -1;
                }
            }

            i_grid++;
            current_length += param.grid_resolution;
        }
    }

    // Find initial laneTree
    int i_tree_start = -1;
    double dist, dist_start = SP_INFINITY;
    for(int i_tree = 0; i_tree < laneTree.size(); i_tree++) {
        if (laneTree[i_tree].id > 0) {
            break;
        }
        dist = (laneTree[i_tree].midPoint - currentPoint).norm();
        if(!p_base->isOccupied(laneTree[i_tree].midPoint, currentPoint)){
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

    // DFS to find midPoints
    int i_tree = laneTree.size() - 1;
    std::vector<int> tail = laneTreeDFS(i_tree, i_tree_start);

    if(tail[0] == -1){
        return false; //TODO: failsafe
    }

//    std::vector<Vector2d> midPoints, leftPoints, rightPoints;
//    midPoints.resize(tail.size()+1);
//    leftPoints.resize(tail.size()+1);
//    rightPoints.resize(tail.size()+1);
//    midPoints[0] = currentPoint;
//    for(int i_tail = 0; i_tail < tail.size(); i_tail++){
//        midPoints[i_tail+1] = laneTree[tail[i_tail]].midPoint;
//        leftPoints[i_tail+1] = laneTree[tail[i_tail]].leftPoint;
//        rightPoints[i_tail+1] = laneTree[tail[i_tail]].rightPoint;
//    }

    std::vector<Vector2d> midPoints, leftPoints, rightPoints;
    midPoints.resize(tail.size());
    leftPoints.resize(tail.size());
    rightPoints.resize(tail.size());
    for(int i_tail = 0; i_tail < tail.size(); i_tail++){
        midPoints[i_tail] = laneTree[tail[i_tail]].midPoint;
        leftPoints[i_tail] = laneTree[tail[i_tail]].leftPoint;
        rightPoints[i_tail] = laneTree[tail[i_tail]].rightPoint;
    }

    // Find midAngles
    std::vector<double> midAngles;
    midAngles.resize(midPoints.size()-1);
    double angle;
    for(int i_angle = 0; i_angle < midAngles.size(); i_angle++) {
        delta = midPoints[i_angle+1] - midPoints[i_angle];
        angle = atan2(delta.y(), delta.x());
        if(angle < 0) {
            angle = angle + 2 * M_PI;
        }
        midAngles[i_angle] = angle;
    }

    // Smoothing using linear interpolation
//    int idx_start, idx_end, idx_delta;
//    Vector2d left_margin, right_margin, l2r;
//    std::vector<Vector2d> smoothing_points;
//    for (int i_smooth = 0; i_smooth < param.max_smoothing_iteration; i_smooth++) {
//        for(int i_mid = 0; i_mid < midPoints.size(); i_mid++) {
//            idx_start = max(i_mid - i_smooth, 0);
//            idx_end = min(i_mid+3+i_smooth, (int)midPoints.size()-1);
//            idx_delta = idx_end - idx_start;
//            if(idx_delta < 1){
//                break;
//            }
//            smoothing_points.resize(idx_delta - 1);
//
//            for(int j_smooth = 0; j_smooth < idx_delta-1; j_smooth++){
//                alpha = static_cast<double>(j_smooth+1)/static_cast<double>(idx_delta);
//                smoothing_points[j_smooth] = (1-alpha) * midPoints[idx_start] + alpha * midPoints[idx_end];
//                left_margin = smoothing_points[j_smooth] - leftPoints[idx_start + j_smooth + 1];
//                right_margin = smoothing_points[j_smooth] - rightPoints[idx_start + j_smooth + 1];
//                l2r = rightPoints[idx_start + j_smooth + 1] - leftPoints[idx_start + j_smooth + 1];
//                if(left_margin.norm() < param.smoothing_margin || left_margin.dot(-l2r) >= 0){
//                    smoothing_points[j_smooth] = leftPoints[idx_start + j_smooth + 1] + l2r.normalized() * param.smoothing_margin;
//                }
//                else if(right_margin.norm() < param.smoothing_margin || right_margin.dot(l2r) >= 0){
//                    smoothing_points[j_smooth] = rightPoints[idx_start + j_smooth + 1] - l2r.normalized() * param.smoothing_margin;
//                }
//            }
//            for(int j_smooth = 0; j_smooth < idx_delta-1; j_smooth++) {
//                midPoints[idx_start + j_smooth + 1] = smoothing_points[j_smooth];
//            }
//        }
//    }

    int idx_start, idx_end, idx_delta, i_angle = 0;
    Vector2d smoothingPoint;
    while(i_angle < midAngles.size()-1) {
        double diff = abs(midAngles[i_angle+1] - midAngles[i_angle]);
        if(diff > M_PI) {
            diff = 2 * M_PI - diff;
        }

        if(diff > param.max_steering_angle) {
            idx_start = i_angle;
            idx_end = min(i_angle+3, (int)midPoints.size());
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
            i_angle = max(i_angle-1, 0);
        }
        else{
            i_angle++;
        }
    }

    // Time allocation
    double nominal_speed = 1; //TODO: nominal speed allocation
    std::vector<double> ts;
    ts.resize(midPoints.size());
    ts[0] = (midPoints[0] - currentPoint).norm() / nominal_speed;
    for(int i_mid = 1; i_mid < midPoints.size(); i_mid++){
        ts[i_mid] = ts[i_mid-1] + (midPoints[i_mid] - midPoints[i_mid-1]).norm() / nominal_speed; //TODO: nominal speed allocation
//        ts[i_mid] = ts[i_mid-1] + (midPoints[i_mid] - midPoints[i_mid-1]).norm() / p_base->nominal_speed; //TODO: fix to this!
    }


    SmoothLane smoothLane;
    smoothLane.points = midPoints;
    smoothLane.ts = ts;

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

std::vector<int> GlobalPlanner::findParents(int id, Vector2d mid_point){
    std::vector<int> parents;
    if(id == 0){
        return parents;
    }

    double shortestDist = SP_INFINITY;
    int i_tree = laneTree.size()-1;
    while(i_tree >= 0 && laneTree[i_tree].id >= id-1){
        if(laneTree[i_tree].id == id){
            i_tree--;
            continue;
        }
        Vector2d parent_point = laneTree[i_tree].midPoint;
        if(!p_base->isOccupied(parent_point, mid_point)){
            double dist = (parent_point-mid_point).norm();
            if(parents.empty()){
                parents.emplace_back(i_tree);
                shortestDist = dist;
            }
            else if(dist < shortestDist){
                parents.insert(parents.begin(), i_tree);
                shortestDist = dist;
            }
            else{
                parents.emplace_back(i_tree);
            }
        }
        i_tree--;
    }
    return parents;
}

std::vector<int> GlobalPlanner::laneTreeDFS(int i_tree, int i_start){
    std::vector<int> tail;
    if(i_tree == i_start){
        tail.emplace_back(i_start);
        return tail;
    }
    else if(laneTree[i_tree].id == 0 || laneTree[i_tree].parents.empty()){
        tail.emplace_back(-1);
        return tail;
    }

    for(int i_parents = 0; i_parents < laneTree[i_tree].parents.size(); i_parents++){
        std::vector<int> tail_prev = laneTreeDFS(laneTree[i_tree].parents[i_parents], i_start);
        if(tail_prev[0] == -1){
            continue;
        }
        else{
            tail = tail_prev;
            tail.emplace_back(i_tree);
            return tail;
        }
    }

    tail.clear();
    tail.emplace_back(-1);
    return tail;
}