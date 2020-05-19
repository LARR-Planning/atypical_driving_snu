#include <atypical_planner/GlobalPlanner.h>
#include <third_party/jps.h>
#include <third_party/parser.h>

#define SP_EPSILON          1e-9
#define SP_EPSILON_FLOAT    1e-4
#define SP_INFINITY         1e+9
#define PI                  3.1415

using namespace Planner;

GlobalPlanner::GlobalPlanner(const Planner::ParamGlobal &g_param,
                             shared_ptr<PlannerBase> p_base_) : AbstractPlanner(p_base_),param(g_param) {
    printf("[GlobalPlanner] Init.\n");

    // initialize grid
    grid_x_min = ceil((param.world_x_min + SP_EPSILON) / param.grid_resolution) * param.grid_resolution;
    grid_y_min = ceil((param.world_y_min + SP_EPSILON) / param.grid_resolution) * param.grid_resolution;
    grid_x_max = floor((param.world_x_max - SP_EPSILON) / param.grid_resolution) * param.grid_resolution;
    grid_y_max = floor((param.world_y_max - SP_EPSILON) / param.grid_resolution) * param.grid_resolution;

    dimx = (int) round((grid_x_max - grid_x_min) / param.grid_resolution) + 1;
    dimy = (int) round((grid_y_max - grid_y_min) / param.grid_resolution) + 1;

    has_wall = false;

    //Parsing
    parser parse_tool;
    parse_tool.get_Coorddata("catkin_ws/src/atypical_driving_snu/keti_pangyo_path3.csv");
    parse_tool.display_result();
    // lanePath = 
    lanePath = parse_tool.get_lanepath();

    //TODO: navigation planning
    {
        // LaneNode l1, l2;
        // int N1 = 20, N2 = 10;

        // l1.width = 10;
        // l2.width = 13;

        // VectorXf l1X(N1);
        // l1X.setZero();
        // VectorXf l1Y(N1);
        // l1Y.setLinSpaced(N1, 0, 73);
        // VectorXf l2X(N2);
        // l2X.setLinSpaced(N2, 0, 49);
        // VectorXf l2Y(N2);
        // l2Y.setConstant(73);

        // for (int n = 0; n < N1; n++) {
        //     geometry_msgs::Point pnt;
        //     pnt.x = l1X(n);
        //     pnt.y = l1Y(n);
        //     l1.laneCenters.push_back(pnt);
        // }

        // for (int n = 0; n < N2; n++) {
        //     geometry_msgs::Point pnt;
        //     pnt.x = l2X(n);
        //     pnt.y = l2Y(n);
        //     l2.laneCenters.push_back(pnt);
        // }
        // lanePath.lanes.emplace_back(l1);
        // lanePath.lanes.emplace_back(l2);
        has_wall = true;
    }
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
//    printf("[GlobalPlanner] planning... \n");
//    auto tCkp = chrono::steady_clock::now(); // check point time
    curSkeletonPath.clear();
    curCorridorSeq.clear();
    grid.clear();
    isFeasible = false;

    //set wall from lanePath
    vector<Point> left, right;
    Point left_point, right_point;
    double d, theta, theta_curr, theta_prev;
    for(auto laneNode: lanePath.lanes){
        d = laneNode.width / 2;
        for(int iter = 0; iter < laneNode.laneCenters.size(); iter++){
            auto lanePoint = laneNode.laneCenters[iter];
            if(iter < laneNode.laneCenters.size() - 1){
                auto nextPoint = laneNode.laneCenters[iter+1];
                theta_prev = theta_curr;
                theta_curr = atan2(nextPoint.y - lanePoint.y, nextPoint.x - lanePoint.x);
            }
            else{
                theta_curr = theta_prev;
            }
            if(iter == 0) {
                theta = theta_curr;
            }
            else{
                theta = (theta_curr + theta_prev) / 2;
            }

            left_point.x = lanePoint.x + d * cos(theta + PI / 2);
            left_point.y = lanePoint.y + d * sin(theta + PI / 2);
            left.emplace_back(left_point);

            right_point.x = lanePoint.x + d * cos(theta - PI / 2);
            right_point.y = lanePoint.y + d * sin(theta - PI / 2);
            right.emplace_back(right_point);
        }
    }

    int left_i = 0;
    while(left_i < left.size() - 1){
        for(int left_j = left_i + 2; left_j < left.size() - 1; left_j++){
            if(intersect(left[left_i], left[left_i+1], left[left_j], left[left_j+1])){
                left.erase(left.begin() + left_i+1, left.begin() + left_j+1);
                break;
            }
        }
        left_i++;
    }

    int right_i = 0;
    while(right_i < right.size() - 1){
        for(int right_j = right_i + 2; right_j < right.size() - 1; right_j++){
            if(intersect(right[right_i], right[right_i+1], right[right_j], right[right_j+1])){
                right.erase(right.begin() + right_i+1, right.begin() + right_j+1);
                break;
            }
        }
        right_i++;
    }

    double point_x, point_y, point_dx, point_dy, step;
    for(int i = 0; i < left.size()-1; i++){
        point_dx = left[i+1].x - left[i].x;
        point_dy = left[i+1].y - left[i].y;

        if(abs(point_dx) >= abs(point_dy)){
            step = abs(point_dx) * 10;
        }
        else{
            step = abs(point_dy) * 10;
        }
        point_dx = point_dx / step;
        point_dy = point_dy / step;

        point_x = left[i].x;
        point_y = left[i].y;

        for(int iter = 0; iter < step; iter++) {
            octomap::point3d point(point_x, point_y, (param.car_z_min + param.car_z_max) / 2);
            p_base->getLocalOctoPtr()->updateNode(point, true);
            point_x += point_dx;
            point_y += point_dy;
        }
    }
    for(int i = 0; i < right.size()-1; i++){
        point_dx = right[i+1].x - right[i].x;
        point_dy = right[i+1].y - right[i].y;

        if(abs(point_dx) >= abs(point_dy)){
            step = abs(point_dx) * 10;
        }
        else{
            step = abs(point_dy) * 10;
        }
        point_dx = point_dx / step;
        point_dy = point_dy / step;

        point_x = right[i].x;
        point_y = right[i].y;

        for(int iter = 0; iter < step; iter++) {
            octomap::point3d point(point_x, point_y, (param.car_z_min + param.car_z_max) / 2);
            p_base->getLocalOctoPtr()->updateNode(point, true);
            point_x += point_dx;
            point_y += point_dy;
        }
    }

    // Set start, goal points
    int i_start, j_start, i_goal, j_goal;
    i_start = (int) round((p_base->getCarState().x - grid_x_min) / param.grid_resolution);
    j_start = (int) round((p_base->getCarState().y - grid_y_min) / param.grid_resolution);
    i_goal = (int) round((p_base->getDesiredState().x - grid_x_min) / param.grid_resolution);
    j_goal = (int) round((p_base->getDesiredState().y - grid_y_min) / param.grid_resolution);

    if(i_start < 0 || i_start >= dimx || j_start < 0 || j_start >= dimy){
        printf("[GlobalPlanner] ERROR: start point is out of bound \n");
        isFeasible = false;
        return false;
    }
    if(i_goal < 0 || i_goal >= dimx || j_goal < 0 || j_goal >= dimy){
        printf("[GlobalPlanner] ERROR: goal point is out of bound \n");
        isFeasible = false;
        return false;
    }

    // Set obstacle
    grid.resize(dimx);
    for(int i = 0; i < dimx; i++){
        grid[i].resize(dimy);
    }

    double car_radius = param.car_width / 2;
    double x, y;
    std::array<double, 4> box;
    int grid_horizon = static_cast<int>(param.horizon / param.grid_resolution * param.car_speed);
    int i_min, i_max, j_min, j_max;
    i_min = max(i_start - grid_horizon, 0);
    j_min = max(j_start - grid_horizon, 0);
    i_max = min(i_start + grid_horizon, dimx - 1);
    j_max = min(j_start + grid_horizon, dimy - 1);

    search_range.xl = i_min * param.grid_resolution + grid_x_min;
    search_range.yl = j_min * param.grid_resolution + grid_y_min;
    search_range.xu = i_max * param.grid_resolution + grid_x_min;
    search_range.yu = j_max * param.grid_resolution + grid_y_min;

    for (int i = i_min; i <= i_max; i++) {
        for (int j = j_min; j <= j_max; j++) {
            x = i * param.grid_resolution + grid_x_min;
            y = j * param.grid_resolution + grid_y_min;

            if (x - car_radius < param.world_x_min) {
                box[0] = param.world_x_min;
            } else {
                box[0] = x - car_radius;
            }
            if (y - car_radius < param.world_y_min) {
                box[1] = param.world_y_min;
            } else {
                box[1] = y - car_radius;
            }
            if (x + car_radius > param.world_x_max) {
                box[2] = param.world_x_max;
            } else {
                box[2] = x + car_radius;
            }
            if (y + car_radius > param.world_y_max) {
                box[3] = param.world_y_max;
            } else {
                box[3] = y + car_radius;
            }

            for (octomap::OcTree::leaf_bbx_iterator it = p_base->getLocalOctoPtr()->begin_leafs_bbx(
                    octomap::point3d(box[0], box[1], param.car_z_min),
                    octomap::point3d(box[2], box[3], param.car_z_max)),
                         end = p_base->getLocalOctoPtr()->end_leafs_bbx(); it != end; ++it) {
                if (p_base->getLocalOctoPtr()->isNodeOccupied(*it)) {
                    grid[i][j] = 1;
                    break;
                }
            }
        }
    }

//    {
//        std::shared_ptr<DynamicEDTOctomap> distmap_obj;
//        float maxDist = 1;
//        octomap::point3d min_point3d(param.world_x_min, param.world_y_min, 0);
//        octomap::point3d max_point3d(param.world_x_max, param.world_y_max, 2);
//        distmap_obj.reset(new DynamicEDTOctomap(maxDist, p_base->getLocalOctoPtr(), min_point3d, max_point3d, false));
//        distmap_obj.get()->update();
//
//        for (int i = 0; i < dimx; i++) {
//            for (int j = 0; j < dimy; j++) {
//                x = i * param.grid_resolution + grid_x_min;
//                y = j * param.grid_resolution + grid_y_min;
//
//                octomap::point3d cur_point(x, y, (param.car_z_max + param.car_z_min)/2);
//                float dist = distmap_obj.get()->getDistance(cur_point);
//                if (dist < 0) {
//                    printf("[GlobalPlanner] distmap error");
//                    return false;
//                }
//
//                if (dist < car_radius) {
//                    grid[i][j] = 1;
//                }
//            }
//        }
//    }
//    cout << "[GlobalPlanner] grid generation: " << std::chrono::duration_cast<std::chrono::microseconds>(chrono::steady_clock::now() - tCkp).count()/1000.0 << "ms" << endl;

    //// JPS start ////
    JPS::Node start(i_start,j_start,0,0,0,0);
    JPS::Node goal(i_goal,j_goal,0,0,0,0);

    start.id_ = start.x_ * dimy + start.y_;
    start.pid_ = start.x_ * dimy + start.y_;
    goal.id_ = goal.x_ * dimy + goal.y_;
    start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);

    if(grid[start.x_][start.y_] == 1 ){
        printf("[GlobalPlanner] ERROR: start point is occluded \n");
        isFeasible = false;
        return false;
    }
    if(grid[goal.x_][goal.y_] == 1){
        printf("[GlobalPlanner] ERROR: start point is occluded \n");
        isFeasible = false;
        return false;
    }

    JPS::JumpPointSearch new_jump_point_search;
    std::vector<JPS::Node> jps_result = new_jump_point_search.jump_point_search(grid, start, goal);

    if(jps_result[0] == JPS::Node(-1,-1,-1,-1,-1,-1)){
        printf("[GlobalPlanner] ERROR: A* failed, there is no path \n");
        isFeasible = false;
        return false;
    }

    for(int i = 0; i < jps_result.size(); i++){
        Point skeletonPoint;
        if(i == jps_result.size() - 1) {
            skeletonPoint.x = jps_result[i].x_ * param.grid_resolution + grid_x_min;
            skeletonPoint.y = jps_result[i].y_ * param.grid_resolution + grid_y_min;
            curSkeletonPath.emplace_back(skeletonPoint);
        }
        else {
            double delta = jps_result[i + 1].cost_ - jps_result[i].cost_;
            for (int j = 0; j < delta; j++) {
                skeletonPoint.x = ((double) jps_result[i].x_ * (1 - j / delta) + (double) jps_result[i + 1].x_ * j / delta) *
                             param.grid_resolution + grid_x_min;
                skeletonPoint.y = ((double) jps_result[i].y_ * (1 - j / delta) + (double) jps_result[i + 1].y_ * j / delta) *
                        param.grid_resolution + grid_y_min;
                curSkeletonPath.emplace_back(skeletonPoint);
            }
        }
    }
     //// JPS finish ////

    //// Corridor Generation Start ////
    double x_curr, y_curr, x_next, y_next, dx, dy;
    std::array<double, 4> box_prev = {0,0,0,0};
    int M = min((int)(param.horizon / param.grid_resolution * param.car_speed), (int)curSkeletonPath.size() - 1);
//    int M = curSkeletonPath.size() - 1;
    for (int m = 0; m < M; m++) {
        auto state_curr = curSkeletonPath[m];
        x_curr = state_curr.x;
        y_curr = state_curr.y;

        std::array<double, 4> box_curr;
        auto state_next = curSkeletonPath[m + 1];
        x_next = state_next.x;
        y_next = state_next.y;

        if (x_next > box_prev[0] - SP_EPSILON && y_next > box_prev[1] - SP_EPSILON
            && x_next <= box_prev[2] + SP_EPSILON && y_next < box_prev[3] + SP_EPSILON) {
            continue;
        }

        // Initialize box
        box_curr[0] = round(std::min(x_curr, x_next) / param.box_resolution) * param.box_resolution;
        box_curr[1] = round(std::min(y_curr, y_next) / param.box_resolution) * param.box_resolution;
        box_curr[2] =
                round(std::max(x_curr, x_next) / param.box_resolution) * param.box_resolution + param.box_resolution;
        box_curr[3] =
                round(std::max(y_curr, y_next) / param.box_resolution) * param.box_resolution + param.box_resolution;
//        box.emplace_back(round(std::max(x,x_next) / param.box_xy_res) * param.box_xy_res);
//        box.emplace_back(round(std::max(y,y_next) / param.box_xy_res) * param.box_xy_res); //TODO: consider this

        // Check initial box
        for (octomap::OcTree::leaf_bbx_iterator it = p_base->getLocalOctoPtr()->begin_leafs_bbx(
                octomap::point3d(box_curr[0], box_curr[1], param.car_z_min),
                octomap::point3d(box_curr[2], box_curr[3], param.car_z_max)),
                     end = p_base->getLocalOctoPtr()->end_leafs_bbx(); it != end; ++it) {
            if (p_base->getLocalOctoPtr()->isNodeOccupied(*it)) {
                printf("[GlobalPlanner] ERROR: Invalid initial trajectory. Obstacle invades initial trajectory");
                return false;
            }
        }

        // Expand box
        double car_radius = param.car_width / 2;
        std::array<double, 4> box_cand, box_update;
        std::vector<int> axis_cand{0, 1, 2, 3};
        int axis, i = -1;
        while (!axis_cand.empty()) {
            box_cand = box_curr;
            box_update = box_curr;
            //check update_box only! update_box + current_box = cand_box
            while (box_update[0] > param.world_x_min - SP_EPSILON
                   && box_update[1] > param.world_y_min - SP_EPSILON
                   && box_update[2] < param.world_x_max + SP_EPSILON
                   && box_update[3] < param.world_y_max + SP_EPSILON
                   && box_cand[2] - box_cand[0] < param.box_max_size
                   && box_cand[3] - box_cand[1] < param.box_max_size) {
                bool isObstacleInBox = false;
                for (octomap::OcTree::leaf_bbx_iterator it = p_base->getLocalOctoPtr()->begin_leafs_bbx(
                        octomap::point3d(box_update[0], box_update[1], param.car_z_min),
                        octomap::point3d(box_update[2], box_update[3], param.car_z_max)),
                             end = p_base->getLocalOctoPtr()->end_leafs_bbx(); it != end; ++it) {
                    if (p_base->getLocalOctoPtr()->isNodeOccupied(*it)) {
                        isObstacleInBox = true;
                        break;
                    }
                }
                if (isObstacleInBox) {
                    break;
                }

                // axis select
                i++;
                if (i >= axis_cand.size()) {
                    i = 0;
                }
                axis = axis_cand[i];

                //update current box
                box_curr = box_cand;
                box_update = box_cand;

                //expand cand_box and get updated part of box(update_box)
                if (axis < 2) {
                    box_update[axis + 2] = box_cand[axis];
                    box_cand[axis] = box_cand[axis] - param.box_resolution;
                    box_update[axis] = box_cand[axis];
                } else {
                    box_update[axis - 2] = box_cand[axis];
                    box_cand[axis] = box_cand[axis] + param.box_resolution;
                    box_update[axis] = box_cand[axis];
                }
            }
            axis_cand.erase(axis_cand.begin() + i);
            if (i > 0) {
                i--;
            } else {
                i = axis_cand.size() - 1;
            }
        }

//        for (int j = 0; j < 1; j++) box_curr[j] += car_radius;
//        for (int j = 2; j < 3; j++) box_curr[j] -= car_radius; //TODO: uncomment here

        Corridor corridor = {box_curr[0], box_curr[1], box_curr[2], box_curr[3], -1, -1};
        curCorridorSeq.emplace_back(corridor);
        box_prev = box_curr;
    }

    // Generate box time segment
    int box_max = curCorridorSeq.size();
    int path_max = M + 1;
    std::vector<std::vector<int>> box_log(box_max);
    for(int i = 0; i < box_max; i++){
        box_log[i].resize(path_max);
    }

    for (int i = 0; i < box_max; i++) {
        for (int j = 0; j < path_max; j++) {
            x = curSkeletonPath[j].x;
            y = curSkeletonPath[j].y;
            if (x > curCorridorSeq[i].xl - SP_EPSILON
                && y > curCorridorSeq[i].yl - SP_EPSILON
                && x < curCorridorSeq[i].xu + SP_EPSILON
                && y < curCorridorSeq[i].yu + SP_EPSILON)
            {
                if (j == 0) {
                    box_log[i][j] = 1;
                } else {
                    box_log[i][j] = box_log[i][j - 1] + 1;
                }
            }
        }
    }

    double timeSegment;
    int current_index, prev_index = 0;
    int box_iter = 0;
    for (int path_iter = 0; path_iter < path_max; path_iter++) {
        if (box_iter == box_max - 1) {
            if (box_log[box_iter][path_iter] > 0) {
                continue;
            } else {
                box_iter--;
            }
        }
        if (box_log[box_iter][path_iter] > 0 && box_log[box_iter+1][path_iter] > 0) {
            int count = 1;
            while (path_iter + count < path_max && box_log[box_iter][path_iter + count] > 0
                   && box_log[box_iter + 1][path_iter + count] > 0) {
                count++;
            }
            current_index = path_iter + count / 2;

            timeSegment = 0;
            double delta_x, delta_y, box_width, time_coefficient;
            for(int i = prev_index; i < current_index; i++){
                delta_x = abs(curSkeletonPath[i+1].x - curSkeletonPath[i].x);
                delta_y = abs(curSkeletonPath[i+1].y - curSkeletonPath[i].y);
                if(delta_x > SP_EPSILON_FLOAT && delta_y < SP_EPSILON_FLOAT){
                    box_width = min(curCorridorSeq[box_iter].yu - curCorridorSeq[box_iter].yl, param.road_width);
                    time_coefficient = param.road_width / box_width; //TODO: naive time_coefficient formulation
                    timeSegment += time_coefficient * delta_x / param.car_speed;
                }
                else if(delta_x < SP_EPSILON_FLOAT && delta_y > SP_EPSILON_FLOAT){
                    box_width = min(curCorridorSeq[box_iter].xu - curCorridorSeq[box_iter].xl, param.road_width);
                    time_coefficient = box_width / param.road_width; //TODO: naive time_coefficient formulation
                    timeSegment += time_coefficient * delta_y / param.car_speed;
                }
                else if(delta_x < SP_EPSILON_FLOAT && delta_y < SP_EPSILON_FLOAT){
                    timeSegment += param.grid_resolution / param.car_speed;
                }
                else{
                    printf("[GlobalPlanner] ERROR: Invalid initial trajectory. initial trajectory has diagonal move");
                    return false;
                }
            }
            curCorridorSeq[box_iter].t_end = timeSegment;

            path_iter = current_index;
            prev_index = current_index;
            box_iter++;
        } else if (box_log[box_iter][path_iter] == 0) {
            box_iter--;
            path_iter--;
        }
    }

    curCorridorSeq[0].t_start = 0;
    for(int i = 1; i < box_max; i++) {
        curCorridorSeq[i].t_end = curCorridorSeq[i-1].t_end + curCorridorSeq[i].t_end;
        curCorridorSeq[i].t_start = curCorridorSeq[i-1].t_end;
    }
    curCorridorSeq[box_max - 1].t_end = SP_INFINITY;
    //// Corridor Generation Finish ////

//    printf("[GlobalPlanner] Done. \n");

    //TODO: print out the outcome of the planning

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

/**
 * @brief update the global planning result to the shared resource
 */
void GlobalPlanner::updateCorridorToBase() {

    // update routine here
    p_base->setCorridorSeq(curCorridorSeq); // just an example
    p_base->setSkeletonPath(curSkeletonPath); //TODO: delete this after debugging
    p_base->setSearchRange(search_range);
}