//
// Created by jbs on 20. 4. 6..
//
#ifndef ATYPICAL_DRIVING_LOCALPLANNER_H
#define ATYPICAL_DRIVING_LOCALPLANNER_H

#include <atypical_planner/PlannerCore.h>
#include <optimization_module/problem_description.h>
#include <optimization_module/ilqr.hpp>

///* YW Added*/
//template<typename T, const int size>
//using Collection = std::array<T,size>;

namespace Planner {

    /**
     * The abstact class of local planner. This class cannot be instantiated.
     */
    class LocalPlanner: public AbstractPlanner {
    protected:
        ParamLocal param;
        
           // Planning intermediate outputs
        MPCResultTraj curPlanning;
        Collection<Corridor,51> box_constraint;
    public:
        LocalPlanner(const ParamLocal& l_param,shared_ptr<PlannerBase> p_base_);
        void updateTrajToBase();
        void SfcToOptConstraint();
        Collection<Corridor,51> getOptCorridor();
    };
    /**
     * Plain MPC module
     */
    class LocalPlannerPlain : public LocalPlanner{
    // private:
    //     Collection<Corridor,l_param.> curCorridorSeq;


    public:
        LocalPlannerPlain(const ParamLocal& l_param,shared_ptr<PlannerBase> p_base_);
        bool plan() override;
    };
    /**
     * Stochastic MPC module
     */
    class LocalPlannerStochastic : public LocalPlanner{
    private:

    public:
        LocalPlannerStochastic(const ParamLocal& l_param,shared_ptr<PlannerBase> p_base_);
        bool plan() override;
    };
}
#endif //ATYPICAL_DRIVING_LOCALPLANNER_H
