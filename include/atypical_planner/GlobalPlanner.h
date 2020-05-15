//
// Created by jbs on 20. 4. 6..
//

#ifndef ATYPICAL_DRIVING_GLOBALPLANNER_H
#define ATYPICAL_DRIVING_GLOBALPLANNER_H

#include <atypical_planner/PlannerCore.h>
#include <third_party/jps.h>

namespace Planner{
    class GlobalPlanner: public AbstractPlanner{
    private:
        ParamGlobal param;
        double grid_x_min, grid_y_min, grid_x_max, grid_y_max;
        int dimx, dimy;
        std::vector<std::vector<int>> grid;

        // Planning intermediate outputs
//        vector<CarState> navigationPath;
        vector<Corridor> curCorridorSeq;
        vector<pair<double, double>> curSkeletonPath; //TODO: delete this after debugging
        Corridor search_range;

    public:
        GlobalPlanner(const ParamGlobal& g_param,shared_ptr<PlannerBase> p_base_);
        bool plan();
        void updateCorridorToBase();
        bool isCurTrajFeasible(); // TODO
        bool intersect(Point i0, Point i1, Point j0, Point j1);
        int ccw(Point a, Point b, Point c);
    };

}


#endif //ATYPICAL_DRIVING_GLOBALPLANNER_H
