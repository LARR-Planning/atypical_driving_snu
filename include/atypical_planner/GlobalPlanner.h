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
            // Planning intermediate outputs

            vector<Corridor> curCorridorSeq;
    public:
        GlobalPlanner(const ParamGlobal& g_param,shared_ptr<PlannerBase> p_base_);
        bool plan();
        void updateCorridorToBase();
    };

}



#endif //ATYPICAL_DRIVING_GLOBALPLANNER_H
