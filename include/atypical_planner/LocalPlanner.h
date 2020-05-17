//
// Created by jbs on 20. 4. 6..
//
#ifndef ATYPICAL_DRIVING_LOCALPLANNER_H
#define ATYPICAL_DRIVING_LOCALPLANNER_H

#include <atypical_planner/PlannerCore.h>
#include <optimization_module/problem_description.h>
#include <optimization_module/ilqr.hpp>
#include <vector>
///* YW Added*/
template<typename T, const int size>
using Collection = std::array<T,size>;

namespace Planner {

    /**
     * The abstact class of local planner. This class cannot be instantiated.
     */
    class LocalPlanner: public AbstractPlanner {
    protected:
        ParamLocal param;
        iLQRParams ilqr_param;
           // Planning intermediate outputs
        MPCResultTraj curPlanning;

        // YW added
        Collection<Corridor,51> box_constraint;
        Collection<Matrix<double,2,2>,51> bodyArray;
        vector<Collection<Matrix<double,2,1>,51>> obs_q; // obstacles' path
        vector<Collection<Matrix<double,2,2>,51>> obs_Q; // obstacles' shape matrices.
        Matrix<double,2,2> carDefaultShape;
        // YW added
        void SfcToOptConstraint(); // translate sfc into box constraints in optimization
        void ObstToConstraint(); // translate obstacle predictions
        void QxFromPrediction(Collection<double,51> mpcPredictionHeads);

        Matrix<double,5,1> state_weight_;
        Matrix<double,2,1> input_weight_;
        Matrix<double,5,1> final_weight_;

    public:
        LocalPlanner(const ParamLocal& l_param,shared_ptr<PlannerBase> p_base_);
        void updateTrajToBase();
        Collection<Corridor,51> getOptCorridor();
        bool isCurTrajFeasible(); // TODO
        Matrix<double,2,1> getLocalGoal();
    };
    /**
     * Plain MPC module
     */
    class LocalPlannerPlain : public LocalPlanner{
    // private:
    //     Collection<Corridor,l_param.> curCorridorSeq;


    public:
        LocalPlannerPlain(const ParamLocal& l_param,shared_ptr<PlannerBase> p_base_);
        bool plan(double t ) override;
    };
    /**
     * Stochastic MPC module
     */
    class LocalPlannerStochastic : public LocalPlanner{
    private:

    public:
        LocalPlannerStochastic(const ParamLocal& l_param,shared_ptr<PlannerBase> p_base_);
        bool plan(double t) override;
    };
}
#endif //ATYPICAL_DRIVING_LOCALPLANNER_H
