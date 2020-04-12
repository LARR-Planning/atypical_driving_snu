//
// Created by jbs on 20. 4. 11..
//

#include <atypical_planner/LocalPlanner.h>


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





////////////////////////////
//   Local Planner-Plain  //
////////////////////////////

LocalPlannerPlain::LocalPlannerPlain(const Planner::ParamLocal &l_param,
                                     shared_ptr<PlannerBase> p_base_) :LocalPlanner(l_param,p_base_) {
    cout << "[LocalPlanner] Plain MPC mode engaged." << endl;
}

/**
 * @brief Plan with plainMPC. Every elements are deterministic
 * @return
 */
bool LocalPlannerPlain::plan() {
    cout << "[LocalPlanner] planning... " << endl;


    cout << "[LocalPlanner] Done. " << endl;

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

    cout << "[LocalPlanner] planning... " << endl;


    cout << "[LocalPlanner] Done. " << endl;

    //TODO: print out the outcome of the planning

    return true; // change this line properly
}








