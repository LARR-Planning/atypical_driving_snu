#include <atypical_planner/GlobalPlanner.h>
#include <third_party/jps.h>
#define SP_EPSILON          1e-9
#define SP_EPSILON_FLOAT    1e-4
#define SP_INFINITY         1e+9

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

//    //TODO: delete below test-purpose code
//    navigationPath.clear();
//    for(int i = -26; i >= -68; i--){
//        CarState fake_navigation_point = {(double)i, 4.5, 0, 0};
//        navigationPath.emplace_back(fake_navigation_point);
//    }
//    for(int i = 5; i <= 20; i++){
//        CarState fake_navigation_point = {-68, (double)i, 0, 0};
//        navigationPath.emplace_back(fake_navigation_point);
//    }
}

/**
 * @brief Global planning routine
 * @return true if success
 */
bool GlobalPlanner::plan() {
    printf("[GlobalPlanner] planning... \n");
    curSkeletonPath.clear();
    curCorridorSeq.clear();
    grid.clear();

    // Set obstacle
    grid.resize(dimx);
    for(int i = 0; i < dimx; i++){
        grid[i].resize(dimy);
    }

    double car_radius = param.car_width / 2;
    double x, y;
    std::array<double, 4> box;
    for (int i = 0; i < dimx; i++) {
        for (int j = 0; j < dimy; j++) {
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
                }
            }
        }
    }

    //// JPS start ////
    // Set start, goal points
    int i_start, j_start, i_goal, j_goal;
    i_start = (int) round((p_base->getCarState().x - grid_x_min) / param.grid_resolution);
    j_start = (int) round((p_base->getCarState().y - grid_y_min) / param.grid_resolution);
    i_goal = (int) round((p_base->getDesiredState().x - grid_x_min) / param.grid_resolution);
    j_goal = (int) round((p_base->getDesiredState().y - grid_y_min) / param.grid_resolution);

    Node start(i_start,j_start,0,0,0,0);
    Node goal(i_goal,j_goal,0,0,0,0);

    start.id_ = start.x_ * dimy + start.y_;
    start.pid_ = start.x_ * dimy + start.y_;
    goal.id_ = goal.x_ * dimy + goal.y_;
    start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);

    if(grid[start.x_][start.y_] == 1 ){
        printf("[GlobalPlanner] ERROR: start point is occluded \n");
        return false;
    }
    if(grid[goal.x_][goal.y_] == 1){
        printf("[GlobalPlanner] ERROR: start point is occluded \n");
        return false;
    }

    JumpPointSearch new_jump_point_search;
    std::vector<Node> jps_result = new_jump_point_search.jump_point_search(grid, start, goal);

    for(int i = 0; i < jps_result.size(); i++){
        pair<double, double> pair;
        if(i == jps_result.size() - 1) {
            pair.first = jps_result[i].x_ * param.grid_resolution + grid_x_min;
            pair.second = jps_result[i].y_ * param.grid_resolution + grid_y_min;
            curSkeletonPath.emplace_back(pair);
        }
        else {
            double delta = jps_result[i + 1].cost_ - jps_result[i].cost_;
            for (int j = 0; j < delta; j++) {
                pair.first = ((double) jps_result[i].x_ * (1 - j / delta) + (double) jps_result[i + 1].x_ * j / delta) *
                             param.grid_resolution + grid_x_min;
                pair.second = ((double) jps_result[i].y_ * (1 - j / delta) + (double) jps_result[i + 1].y_ * j / delta) *
                        param.grid_resolution + grid_y_min;
                curSkeletonPath.emplace_back(pair);
            }
        }
    }

    //// JPS finish ////

    //// Corridor Generation Start ////
    double x_curr, y_curr, x_next, y_next, dx, dy;
    std::array<double, 4> box_prev = {0,0,0,0};
    for (int m = 0; m < curSkeletonPath.size() - 1; m++) {
        auto state_curr = curSkeletonPath[m];
        x_curr = state_curr.first;
        y_curr = state_curr.second;

        std::array<double, 4> box_curr;
        auto state_next = curSkeletonPath[m + 1];
        x_next = state_next.first;
        y_next = state_next.second;

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
                   && box_update[3] < param.world_y_max + SP_EPSILON) {
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

        for (int j = 0; j < 1; j++) box_curr[j] += car_radius;
        for (int j = 2; j < 3; j++) box_curr[j] -= car_radius;

        Corridor corridor = {box_curr[0], box_curr[1], box_curr[2], box_curr[3], -1, -1};
        curCorridorSeq.emplace_back(corridor);
        box_prev = box_curr;
    }

    // Generate box time segment
    int box_max = curCorridorSeq.size();
    int path_max = curSkeletonPath.size();
    std::vector<std::vector<int>> box_log(box_max);
    for(int i = 0; i < box_max; i++){
        box_log[i].resize(path_max);
    }

    for (int i = 0; i < box_max; i++) {
        for (int j = 0; j < path_max; j++) {
            x = curSkeletonPath[j].first;
            y = curSkeletonPath[j].second;
            if (x > curCorridorSeq[i].xl - SP_EPSILON
                && y > curCorridorSeq[i].yl - SP_EPSILON
                && x < curCorridorSeq[i].xu - SP_EPSILON
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
                delta_x = abs(curSkeletonPath[i+1].first - curSkeletonPath[i].first);
                delta_y = abs(curSkeletonPath[i+1].second - curSkeletonPath[i].second);
                if(delta_x > SP_EPSILON_FLOAT && delta_y < SP_EPSILON_FLOAT){
                    box_width = min(curCorridorSeq[box_iter].yu - curCorridorSeq[box_iter].yl, param.road_width);
                    time_coefficient = box_width / param.road_width; //TODO: naive time_coefficient formulation
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

    printf("[GlobalPlanner] Done. \n");

    //TODO: print out the outcome of the planning

    return true; // change this line properly
}

/**
 * @brief update the global planning result to the shared resource
 */
void GlobalPlanner::updateCorridorToBase() {

    // update routine here
    p_base->setCorridorSeq(curCorridorSeq); // just an example
    p_base->setSkeletonPath(curSkeletonPath); //TODO: delete this after debugging
}