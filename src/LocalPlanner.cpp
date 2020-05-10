 //
// Created by jbs on 20. 4. 11..
//

#include <atypical_planner/LocalPlanner.h>
#include <optimization_module/problem.hpp>
#include <optimization_module/dimension.h>

using namespace Planner;

///////////////////////////
//   Local Planner base  //
///////////////////////////


/**
 * @brief Construction of local planner
 * @param l_param local planning paramters
 * @param p_base_ Planning base (input + output)
 * @param m mutex set
 */
LocalPlanner::LocalPlanner(const Planner::ParamLocal &l_param,
                           shared_ptr<PlannerBase> p_base_): AbstractPlanner(p_base_),param(l_param){
    cout << "[LocalPlanner] Init." << endl;
}
/**
 * Update the planning result to p_base
 */
void LocalPlanner::updateTrajToBase() {
    // update routine here
    p_base->setMPCResultTraj(curPlanning); // just an example

}
void LocalPlanner::SfcToOptConstraint(){
    double t_end_= 5.0;
    double t_start_  = 0.0;
    int N_corr = 0;
    int count1 = 0;
    int count2 = 0;
    for(auto s: p_base->getCorridorSeq())
    {
        if(count1*count2 == 0)
        {
            box_constraint[count2] = s;
            count1++;
            count2++;
        }

        t_end_ = s.t_end;
        if(t_end_ >param.horizon)
        {
            for(int i = 0; i< floor((param.horizon-t_start_)/param.ts);i++)
            {
                box_constraint[count2] = s;
                count2++;
            }
        }
        else
        {
            for(int i = 0 ; i< floor((t_end_-t_start_)/param.ts);i++)
            {
                box_constraint[count2] = s;
                count2 ++;
            }
            t_start_ = param.ts * count2;

        }

    }
}
 Collection<Corridor,51> LocalPlanner::getOptCorridor()
 {
    return box_constraint;
 }

 LocalPlannerPlain::LocalPlannerPlain(const Planner::ParamLocal &l_param,
                                      shared_ptr<PlannerBase> p_base_) :LocalPlanner(l_param,p_base_) {
     cout << "[LocalPlanner] Plain MPC mode engaged." << endl;
 }
/**
 * @brief Plan with plainMPC. Every elements are deterministic
 * @return
 */
bool LocalPlannerPlain::plan() {
//    Checked that this function is always executed
//    cout << "[LocalPlanner] planning... " << endl;
//    cout << "[LocalPlanner] Done. " << endl;

    LocalPlanner::SfcToOptConstraint(); // convert SFC to box constraints
    // cout<< LocalPlanner::box_constraint[1].yl<<endl;
    using namespace Eigen;
    //Following codes will be wrapped with another wrapper;

    std::shared_ptr<Problem> prob = std::make_shared<Problem>(box_constraint);
//    // Do not have to be defined every loop
    Matrix<double,Nx,1> state_weight_;
    state_weight_.setZero();
    Matrix<double,Nu,1> input_weight_;
    input_weight_.setZero();
    Matrix<double,Nx,1> final_weight_;
    final_weight_.setZero();
//
    bool noConstraint_ = 0;
    prob->set_state_weight(state_weight_);
    prob->set_final_weight(final_weight_);
    prob->set_input_weight(input_weight_);
    prob->set_noConstraint(noConstraint_);
//
//     // A new goal-update function will be defined/added with argument x_goal
    Matrix<double,2,1> x_goal_;
    x_goal_.setZero();
    prob->set_goal(x_goal_);
    static int loop_num = 0;
    std::array<Matrix<double,Nu,1>,N> u0;
    if(loop_num == 0)
    {
        for(auto &s :u0)
        {
            s=(Matrix<double,Nu,1>()<< 0.0,0.0).finished();
        }
    }

    Matrix<double,Nx,1> x0_new = (Matrix<double,Nx,1>()<<p_base->getCarState().x,
            p_base->getCarState().y,p_base->getCarState().v,p_base->getCurInput().alpha,p_base->getCarState().theta).finished();
    Collection<Matrix<double,Nu,1>,N> uN_new = u0;

    if(loop_num == 0)
    {

    }

    //TODO: print out the outcome of the planning

    return true; // change this line properly
}







/////////////////////////////////
//   Local Planner-Stochastic  //
/////////////////////////////////


LocalPlannerStochastic::LocalPlannerStochastic(const Planner::ParamLocal &l_param,
                                               shared_ptr<PlannerBase> p_base_) :LocalPlanner(l_param,p_base_) {
    cout << "[LocalPlanner] Stochastic MPC mode engaged." << endl;
}

/**
 * @brief Plan with plainMPC. Every elements are deterministic
 * @return
 */
bool LocalPlannerStochastic::plan() {

//    cout << "[LocalPlanner] planning... " << endl;

//    cout << "[LocalPlanner] Done. " << endl;

    //TODO: print out the outcome of the planning

    return true; // change this line properly
}








