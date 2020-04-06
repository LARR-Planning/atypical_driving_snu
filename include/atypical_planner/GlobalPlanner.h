//
// Created by jbs on 20. 4. 6..
//

#ifndef ATYPICAL_DRIVING_GLOBALPLANNER_H
#define ATYPICAL_DRIVING_GLOBALPLANNER_H

#include <atypical_planner/PlannerCore.h>
namespace Planner{
    class GlobalPlanner{
        private:
            PlannerBase* p_base; // planning data
            ParamGlobal param;

    public:
            GlobalPlanner(const ParamGlobal& g_param,PlannerBase* p_base_);
            void update_corridor();
    };

}



#endif //ATYPICAL_DRIVING_GLOBALPLANNER_H
