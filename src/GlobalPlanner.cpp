//
// Created by jbs on 20. 4. 11..
//
#include <atypical_planner/GlobalPlanner.h>

using namespace Planner;

GlobalPlanner::GlobalPlanner(const Planner::ParamGlobal &g_param,
                             shared_ptr<PlannerBase> p_base_) : AbstractPlanner(p_base_),param(g_param) {
    printf("[GlobalPlanner] Init.\n");
};

/**
 * @brief Global planning routine
 * @return true if success
 */
bool GlobalPlanner::plan() {
    printf("[GlobalPlanner] planning... \n");


    printf("[GlobalPlanner] Done. \n");

    //TODO: print out the outcome of the planning

    return true; // change this line properly
}

/**
 * @brief update the global planning result to the shared resource
 */
void GlobalPlanner::updateCorridorToBase() {

    // update routine here
    p_base->setCorridorSeq(curCorridorSeq); // just an example

}