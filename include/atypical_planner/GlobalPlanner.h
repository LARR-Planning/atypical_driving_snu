//
// Created by jbs on 20. 4. 6..
//

#ifndef ATYPICAL_DRIVING_GLOBALPLANNER_H
#define ATYPICAL_DRIVING_GLOBALPLANNER_H

#include <atypical_planner/PlannerCore.h>


namespace Planner{
    class GlobalPlanner: public AbstractPlanner{
    private:
        ParamGlobal param;
        double grid_x_min, grid_y_min, grid_x_max, grid_y_max;
        int dimx, dimy;
        std::vector<std::vector<int>> grid;
        std::array<double, 4> world_box_transformed;
        std::vector<LaneTreeElement> laneTree;
        std::vector<LaneTreeElement> laneTreePath;
        bool isFeasible;

        // Planning intermediate outputs
//        vector<CarState> navigationPath;
        vector<Corridor> curCorridorSeq;
        vector<Point> curSkeletonPath; //TODO: delete this after debugging
        Corridor search_range;

    public:
        GlobalPlanner(const ParamGlobal& g_param,shared_ptr<PlannerBase> p_base_);
        bool plan(double t);
        void updateCorridorToBase();
        bool isCurTrajFeasible(); // TODO
        bool intersect(Point i0, Point i1, Point j0, Point j1);
        int ccw(Point a, Point b, Point c);
        static std::array<double, 4> boxTransform(const SE3& Tab, const std::array<double, 4>& box);
        std::vector<int> findChildren(int idx);
        void laneTreeSearch(int idx);
        std::vector<int> getMidPointsFromLaneTree(int i_tree_start);
        int findLaneTreePathTail(bool& isBlocked, bool& isBlockedByObject);
//        bool isLaneTreeBlocked(int last_element_index);
//        bool isLaneTreeBlockedByDynamicObs(int last_element_index);
//        std::vector<int> cutTail(const std::vector<int>& tail) const;
    };

}


#endif //ATYPICAL_DRIVING_GLOBALPLANNER_H
