//
// Created by jbs on 20. 4. 6..
//

#include <atypical_planner/Wrapper.h>

using namespace Planner;


/////////////////
// Ros Wrapper //
/////////////////

/**
 * Constructor for ros wrapper
 * @param p_base_
 * @param mSet_ mutex set of size 2.
 */
RosWrapper::RosWrapper(shared_ptr<PlannerBase> p_base_,mutex* mSet_):p_base(p_base_),nh("~"),mSet(mSet_){
    // Initiate ros communication (caution: parameters are parsed in udpateParam)

    // Publisher
    pubPath = nh.advertise<nav_msgs::Path>("planning_path",1);

    // Subscriber
    subCarPoseCov = nh.subscribe("car_pose_cov",1,&RosWrapper::cbCarPoseCov,this);
}

/**
 * @brief Fetching the parameter from ros node handle.
 * @param param_ The fetched params will be written in param_
 */
void RosWrapper::updateParam(Param &param_) {
    // Do some parsing here from reading from ros launch
    ROS_INFO("Reading the parameters from launch..");
    nh.param<string>("world_frame_id",worldFrameId,"/map");
    // Own
    planningPath.header.frame_id = worldFrameId;

    // global planner
    nh.param<double>("global_planner/horizon",param_.g_param.horizon,5);

    // local planner
    nh.param<double>("local_planner/horizon",param_.l_param.horizon,5);
}

/**
 * @breif Convert the row information of p_base into rosmsgs
 * @details Extract information from p_base
 */
void RosWrapper::prepareROSmsgs() {

    // 1. Topics directly obtained from p_base
    if(mSet[1].try_lock()){
        // Example below
        planningPath.poses.clear();
        geometry_msgs::PoseStamped poseStamped;
        for (auto pose : p_base->getMPCResultTraj().xs){
            poseStamped.pose.position.x = pose.x;
            poseStamped.pose.position.y = pose.y;
            planningPath.poses.push_back(poseStamped);
        }
        mSet[1].unlock();
    }else{
        ROS_WARN("[RosWrapper] Locking failed for ros data update. The output of p_base is being modified in planner ");
    }

}

/**
 * @brief publish the car input and visualization markers
 * @details Do not use p_base here !!
 */
void RosWrapper::publish() {
    // e.g pub1.publish(topic1)
    pubPath.publish(planningPath);
}

/**
 * @brief While loop of the ros wrapper (publish and spin)
 */
void RosWrapper::runROS() {
        ros::Rate lr(50);
        ros::Time t(ros::Time::now());

        while(ros::ok()){

            prepareROSmsgs(); // you may trigger this only under some conditions
            publish();
            ros::spinOnce(); // callback functions are executed
            lr.sleep();
        }
}

/**
 * @brief receive the car pose Cov and update
 * @param dataPtr
 */
void RosWrapper::cbCarPoseCov(geometry_msgs::PoseWithCovarianceConstPtr dataPtr) {
    // Just an example
    // TODO you have to decide whether the update in this callback could interrupt planning thread
    if(mSet[0].try_lock()){
        CarState curState;
        curState.x = dataPtr->pose.position.x;
        curState.y = dataPtr->pose.position.y;
        p_base->setCarState(curState);
        mSet[0].unlock();
        isCarPoseCovReceived = true;
    }else{
        ROS_WARN("[RosWrapper] callback for CarPoseCov locked by planner. Passing update");
    }
}
/**
 * @brief Whether all the necessary inputs are received
 * @return true if all the necessary inputs are received
 */
bool RosWrapper::isAllInputReceived() {
    return
//            isGlobalMapReceived and
//            isLocalMapReceived and
              isCarPoseCovReceived;
}



////////////////////////////////////
// Wrapper (top of the hierarchy) //
////////////////////////////////////


/**
 * @brief The constructor 1) initiates all the ros communication, 2) feed params to planners
 */
Wrapper::Wrapper() : p_base_shared(make_shared<PlannerBase>()) {

    // 1. Parse all parameters from nh
    ros_wrapper_ptr = new RosWrapper(p_base_shared,mSet);
    ros_wrapper_ptr->updateParam(param);

    // 2. initialize planners
    lp_ptr = new LocalPlannerPlain(param.l_param,p_base_shared); // TODO Stochastic MPC also available
    // cout << p_base_shared.use_count() << endl;
    gp_ptr = new GlobalPlanner(param.g_param,p_base_shared);
    // cout << p_base_shared.use_count() << endl;

}
/**
 * @brief Run the two thread (Planner & ROS)
 */
void Wrapper::run(){
    // Create thread
    threadPlanner = thread(&Wrapper::runPlanning,this); cout << "Thread for planners has been created" << endl;
    threadRosWrapper = thread(&RosWrapper::runROS,ros_wrapper_ptr); cout <<  "Thread for ros-wrapper has been created" << endl;

    // join and exit
    threadRosWrapper.join();
    threadPlanner.join();
};


/**
 * @brief Update the planning result from lp and gp
 * @return True if both planner succeeded
 */

bool Wrapper::plan(){
    mSet[0].lock();
    ROS_INFO( "[Wrapper] Assume that planning takes 1.0 sec. Locking subscription.\n ");
    std::this_thread::sleep_for(std::chrono::duration<double>(1.0));
    bool gpPassed = gp_ptr->plan();
    printf("----------------------------------------------------------------\n");
    bool lpPassed = lp_ptr->plan();
    mSet[0].unlock();
    return (gpPassed and lpPassed);

}

/**
 * @brief Modify p_base with the resultant planning output
 */
void Wrapper::updateToBase() {
    mSet[1].lock();

    ROS_INFO( "[Wrapper] Assume that updating takes 0.5 sec. Locking rosmsg update.\n ");
    std::this_thread::sleep_for(std::chrono::duration<double>(0.5));

    gp_ptr->updateCorridorToBase();
    lp_ptr->updateTrajToBase();
    mSet[1].unlock();
    ROS_INFO( "[Wrapper] p_base updated. Unlocking.\n ");

}


/**
 * @brief Engage the while loop of planners
 */
void Wrapper::runPlanning() {

    ROS_INFO( "[Wrapper] Assuming planning is updated at every 5 sec.\n"); // TODO
    // initial stuffs
    double Tp = 5; // 5 sec
    auto tCkp = chrono::steady_clock::now(); // check point time
    bool doPlan = true; // turn on if we have started the class
    bool isPlanPossible = false;
    bool isPlanSuccess = false;
    while (ros::ok()){
        // First, we have to check all the inputs are received for planning
        isPlanPossible = ros_wrapper_ptr->isAllInputReceived();
        if (isPlanPossible){
            ROS_INFO_ONCE("[Wrapper] start planning!");
            // If planning is triggered
            if (doPlan){
                tCkp = chrono::steady_clock::now(); // check point time
                printf("================================================================\n");
                ROS_INFO( "[Wrapper] Planning started.\n\n"); // TODO

                // Do planning
                isPlanSuccess = plan();
                printf("================================================================\n");

                // Only when the planning results are valid, we update p_base
                // At this step, the prepareROSmsgs() of RosWraper is unavailable
                if (isPlanSuccess){
                    updateToBase();
                }
            }
            // Trigger condition of planning. This can be anything other than the simple periodic triggering
            doPlan = chrono::steady_clock::now() - tCkp > std::chrono::duration<double>(Tp);

        }else{
            ROS_WARN_ONCE("[Wrapper] waiting planning input subscriptions..");
        }
    }
}




