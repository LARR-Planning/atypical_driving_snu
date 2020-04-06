//
// Created by jbs on 20. 4. 6..
//

#include <atypical_planner/Wrapper.h>

using namespace Planner;

/**
 * @brief The constructor 1) initiates all the ros communication, 2) feed params to planners
 */
Wrapper::Wrapper() : p_base_shared(make_shared<PlannerBase>()) {

    // 1. Parse all parameters from nh
    ros_wrapper.updateParam(param);

    // 2. initialize planners
    lp_ptr = new LocalPlanner(param.l_param,p_base_shared.get());
    cout << p_base_shared.use_count() << endl;
    gp_ptr = new GlobalPlanner(param.g_param,p_base_shared.get());
    cout << p_base_shared.use_count() << endl;

}
/**
 * @brief while loop in the main
 */
void Wrapper::run(){

    bool doPlan = true;

    while(ros::ok()){
        // 1. publish
        ros_wrapper.publish();

        // 2. planning trigger checking
        if(doPlan){
            update();
        }
    };
};
/**
 * @brief update two planners
 */
void Wrapper::update(){
    planGlobal();
    planLocal();
}

/**
 * @brief trigger global planning
 */
void Wrapper::planGlobal(){
    gp_ptr->update_corridor();
}

/**
 * @brief trigger local planning
 */
void Wrapper::planLocal(){
    lp_ptr->updateTraj();
}

