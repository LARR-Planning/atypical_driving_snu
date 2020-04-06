//
// Created by jbs on 20. 4. 6..
//

#ifndef ATYPICAL_DRIVING_LOCALPLANNER_H
#define ATYPICAL_DRIVING_LOCALPLANNER_H

#include <atypical_planner/PlannerCore.h>

namespace Planner {

    /**
     * The abstact class of local planner. This class cannot be instantiated.
     */
    class LocalPlanner {
        private:
            PlannerBase* p_base; // planning data
            ParamLocal param;

            // do some planning here...

    public:
            LocalPlanner(const ParamLocal& l_param,PlannerBase* p_base_);
            virtual void updateTraj(); // to be override
    };
    /**
     * Plain MPC module
     */
    class LocalPlannerPlain : public LocalPlanner{
        void updateTraj();
    };
    /**
     * Stochastic MPC module
     */
    class LocalPlannerStochastic : public LocalPlanner{
        void updateTraj();
    };
}
#endif //ATYPICAL_DRIVING_LOCALPLANNER_H
