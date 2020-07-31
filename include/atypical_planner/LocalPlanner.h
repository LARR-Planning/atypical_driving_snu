//
// Created by jbs on 20. 4. 6..
//
#ifndef ATYPICAL_DRIVING_LOCALPLANNER_H
#define ATYPICAL_DRIVING_LOCALPLANNER_H

#include <atypical_planner/PlannerCore.h>
#include <optimization_module/problem_description.h>
#include <optimization_module/ilqr.hpp>
#include <optimization_module/dimension.h>
#include <Eigen/StdVector>
#include <vector>
#include <array>
///* YW Added*/
template<typename T, const int size>
using Collection = std::array<T,size>;

namespace Planner {

    /**
     * The abstact class of local planner. This class cannot be instantiated.
     */
    class LocalPlanner: public AbstractPlanner {

    public:
        LocalPlanner(const ParamLocal& l_param,shared_ptr<PlannerBase> p_base_);
        void updateTrajToBase();
        Collection<Corridor,N+1> getOptCorridor();
        bool isCurTrajFeasible(); // TODO


    protected:
        ParamLocal param; // From yaml file
        iLQRParams ilqr_param; // iLQR parameters
        MPCResultTraj curPlanning; //Planning with intermediate outputs

        vector<vector<Matrix2d>> obs_Q;
        vector<vector<Vector2d>> obs_q;
        Matrix<double,2,2> car_shape;
        Collection<Matrix<double,2,1>,N> uN_NextInit;

        Matrix<double,Nx,1> next_state;
		Matrix<double,Nx,1> cur_state;
        // YW added
        Collection<Corridor,N+1> box_constraint;

        bool isRefPlausible; // check whether reference has garbage value
        bool isSFCPlausible; // check whether SFC has garbage value
        bool isDynObstPlausible; //check whether Dynamics Obstacle has garbage value
        // YW added
        bool SfcToOptConstraint(double t); // translate sfc into box constraints in optimization
        void SetSfcIdx(int N_corr); // choose sfc idx
        bool ObstToConstraint(); // translate obstacle predictions
        bool SetLocalWpts(double t); // find closest 50 lane nodes
        //void QxFromPrediction(Collection<double,N+1> mpcPredictionHeads);        
		int isRefUsed;
        Collection<Matrix<double,5,1>,N+1> local_wpts;

        Collection<bool,N+1> sfc_idx; //  sfc idx


		//Matrix<double,2,1> wpts_initial;
        Matrix<double,Nx,1> state_weight_;
        Matrix<double,Nu,1> input_weight_;
        Matrix<double,Nx,1> final_weight_;

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
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        LocalPlannerStochastic(const ParamLocal& l_param,shared_ptr<PlannerBase> p_base_);
        bool plan(double t) override;
    private:


    };
}
#endif //ATYPICAL_DRIVING_LOCALPLANNER_H
