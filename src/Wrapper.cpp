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
    pubCorridorSeq = nh.advertise<visualization_msgs::MarkerArray>("corridor_seq", 1);

    // Subscriber
    subCarPoseCov = nh.subscribe("car_pose_cov",1,&RosWrapper::cbCarPoseCov,this);
    subDesiredCarPose = nh.subscribe("desired_car_pose",1,&RosWrapper::cbDesiredCarPose,this);
    subGlobalMap = nh.subscribe("global_map",1,&RosWrapper::cbGlobalMap,this);
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
    nh.param<double>("global_planner/car_width",param_.g_param.car_width,2);
    nh.param<double>("global_planner/car_height",param_.g_param.car_z_min,-0.5);
    nh.param<double>("global_planner/car_height",param_.g_param.car_z_max,0.0);
    nh.param<double>("global_planner/car_speed",param_.g_param.car_speed,1.0);
    nh.param<double>("global_planner/road_width",param_.g_param.road_width,4.0);
    nh.param<double>("global_planner/world_x_min",param_.g_param.world_x_min,-100);
    nh.param<double>("global_planner/world_y_min",param_.g_param.world_y_min,-5);
    nh.param<double>("global_planner/world_x_max",param_.g_param.world_x_max,0);
    nh.param<double>("global_planner/world_y_max",param_.g_param.world_y_max,30);
    nh.param<double>("global_planner/grid_resolution",param_.g_param.grid_resolution,0.5);
    nh.param<double>("global_planner/box_resolution",param_.g_param.box_resolution,0.1);

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
        // planning path
        planningPath.poses.clear();
        geometry_msgs::PoseStamped poseStamped;
        for (auto pose : p_base->getMPCResultTraj().xs){
            poseStamped.pose.position.x = pose.x;
            poseStamped.pose.position.y = pose.y;
            planningPath.poses.push_back(poseStamped);
        }

        // corridor_seq jungwon
        corridorSeq.markers.clear();
        int marker_id = 0;
        double car_width = 2;
        double car_z_min = 0.5; //TODO: save car_z when updateParam
        double car_z_max = 0.7; //TODO: save car_z when updateParam
        visualization_msgs::Marker marker;
        marker.header.frame_id = worldFrameId;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color.a = 0.2;
        marker.color.r = 0;
        marker.color.g = 1;
        marker.color.b = 0;
        for(auto corridor : p_base->getCorridorSeq()){
            marker.ns = "corridor";
            marker.id = marker_id++;
            marker.pose.position.x = (corridor.xu + corridor.xl) / 2;
            marker.pose.position.y = (corridor.yu + corridor.yl) / 2;
            marker.pose.position.z = (car_z_min + car_z_max)/2;
            marker.scale.x = corridor.xl - corridor.xu;
            marker.scale.y = corridor.yl - corridor.yu;
            marker.scale.z = car_z_max - car_z_min;
            corridorSeq.markers.emplace_back(marker);
        }
        for(auto node : p_base->getSkeletonPath()){
            marker.ns = "skeleton_path";
            marker.id = marker_id++;
            marker.color.a = 1;
            marker.color.r = 1;
            marker.color.g = 0;
            marker.color.b = 0;
            marker.pose.position.x = node.first;
            marker.pose.position.y = node.second;
            marker.pose.position.z = (car_z_min + car_z_max)/2;
            marker.scale.x = car_width;
            marker.scale.y = car_width;
            marker.scale.z = car_width;
            corridorSeq.markers.emplace_back(marker);
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
    pubCorridorSeq.publish(corridorSeq);
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
 * @brief receive the desired car pose and update
 * @param dataPtr
 */
void RosWrapper::cbDesiredCarPose(geometry_msgs::PoseConstPtr dataPtr) {
    // Just an example
    // TODO you have to decide whether the update in this callback could interrupt planning thread
    if(mSet[0].try_lock()){
        CarState desiredState;
        desiredState.x = dataPtr->position.x;
        desiredState.y = dataPtr->position.y;
        p_base->setDesiredState(desiredState);
        mSet[0].unlock();
        isCarPoseCovReceived = true;
    }else{
        ROS_WARN("[RosWrapper] callback for CarPoseCov locked by planner. Passing update");
    }
}

/**
 * @brief receive the global map and update
 * @param octomap_msg
 */
void RosWrapper::cbGlobalMap(const octomap_msgs::Octomap& octomap_msg) {
    // TODO you have to decide whether the update in this callback could interrupt planning thread
    if(isGlobalMapReceived){
        return;
    }
    if(mSet[0].try_lock()){
        p_base->setGlobalMap(dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(octomap_msg)));
        mSet[0].unlock();
        isGlobalMapReceived = true;
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
              isGlobalMapReceived and
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




