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
    max_marker_id = 0;

    // Publisher
    pubPath = nh.advertise<nav_msgs::Path>("planning_path",1);
    pubCorridorSeq = nh.advertise<visualization_msgs::MarkerArray>("corridor_seq", 1);
    pubObservationMarker = nh.advertise<visualization_msgs::Marker>("observation_queue",1);
    pubPredictionArray = nh.advertise<visualization_msgs::MarkerArray>("prediction",1);
    pubCurCmd = nh.advertise<driving_msgs::VehicleCmd>("/vehicle_cmd",1);


    // Subscriber
    subCarPoseCov = nh.subscribe("/current_pose",1,&RosWrapper::cbCarPoseCov,this);
    subDesiredCarPose = nh.subscribe("desired_car_pose",1,&RosWrapper::cbDesiredCarPose,this);
    subGlobalMap = nh.subscribe("global_map",1,&RosWrapper::cbGlobalMap,this);
    subLocalMap = nh.subscribe("local_map",1,&RosWrapper::cbLocalMap,this);
    subCarSpeed = nh.subscribe("/current_speed",1,&RosWrapper::cbCarSpeed,this);
    subExampleObstaclePose = nh.subscribe("obstacle_pose",1,&RosWrapper::cbObstacles,this);

}
/**
 * @brief update the fitting model only. It does not directly update p_base
 */
void RosWrapper::updatePrediction() {

    p_base->predictorSet[0].update_predict();
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
    nh.param<double>("global_planner/horizon",param_.g_param.horizon,15);
    nh.param<double>("global_planner/car_width",param_.g_param.car_width,2);
    nh.param<double>("global_planner/car_z_min",param_.g_param.car_z_min,0.0);
    nh.param<double>("global_planner/car_z_max",param_.g_param.car_z_max,2.0);
    nh.param<double>("global_planner/car_speed",param_.g_param.car_speed,1.0);
    nh.param<double>("global_planner/world_x_min",param_.g_param.world_x_min,-10);
    nh.param<double>("global_planner/world_y_min",param_.g_param.world_y_min,-1);
    nh.param<double>("global_planner/world_x_max",param_.g_param.world_x_max,35);
    nh.param<double>("global_planner/world_y_max",param_.g_param.world_y_max,80);
    nh.param<double>("global_planner/grid_resolution",param_.g_param.grid_resolution,0.5);
    nh.param<double>("global_planner/box_resolution",param_.g_param.box_resolution,0.3);
    nh.param<double>("global_planner/box_max_size",param_.g_param.box_max_size,10);

    // local planner
    nh.param<double>("local_planner/horizon",param_.l_param.horizon,5);
    nh.param<double>("local_planner/ts",param_.l_param.tStep,0.1);
    nh.param<double>("local_planner/obstacle_radius_nominal",param_.l_param.obstRadiusNominal,0.3);
    nh.param<double>("local_planner/car_longtitude",param_.l_param.carLongtitude,2.7);
    nh.param<double>("local_planner/max_steer",param_.l_param.maxSteer,M_PI/30);
    nh.param<double>("local_planner/max_accel",param_.l_param.maxAccel,3);
    nh.param<double>("local_planner/min_accel",param_.l_param.minAccel,-1);


    // predictor

    nh.param<int>("predictor/observation_queue",param_.p_param.queueSize,6);
    nh.param<float>("predictor/ref_height",param_.p_param.zHeight,1.0);
    nh.param<int>("predictor/poly_order",param_.p_param.polyOrder,1); // just fix 1

    // Common
    double goal_x,goal_y;
    nh.param<double>("initial_goal/x",goal_x,0.0); // just fix 1
    nh.param<double>("initial_goal/y",goal_y,0.0); // just fix 1
    nh.param<double>("goal_thres",param_.l_param.goalReachingThres,0.4); // just fix 1

    ROS_INFO("[SNU_PLANNER/RosWrapper] received goal [%f,%f]",goal_x,goal_y);
    ROS_INFO("[SNU_PLANNER/RosWrapper] Initialized clock with ROS time %f",ros::Time::now().toSec());
    t0 = ros::Time::now().toSec();

    CarState goalState;
    goalState.x = goal_x;
    goalState.y = goal_y;
    p_base->setDesiredState(goalState);

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
        double car_z_min = 0.5; //TODO: save car_z when updateParam
        double car_z_max = 1.5; //TODO: save car_z when updateParam

        visualization_msgs::Marker marker;
        marker.header.frame_id = worldFrameId;
        marker.type = visualization_msgs::Marker::CUBE;

        for(auto corridor : p_base->getCorridorSeq()){
            marker.id = marker_id;
            if(marker_id > max_marker_id){
                marker.action = visualization_msgs::Marker::ADD;
                max_marker_id = marker_id;
            } else{
                marker.action = visualization_msgs::Marker::MODIFY;
            }

            marker.color.a = 0.2;
            marker.color.r = 0;
            marker.color.g = 1;
            marker.color.b = 0;
            marker.pose.position.x = (corridor.xu + corridor.xl) / 2;
            marker.pose.position.y = (corridor.yu + corridor.yl) / 2;
            marker.pose.position.z = (car_z_min + car_z_max)/2;
            marker.scale.x = corridor.xu - corridor.xl;
            marker.scale.y = corridor.yu - corridor.yl;
            marker.scale.z = car_z_max - car_z_min;
            corridorSeq.markers.emplace_back(marker);
            marker_id++;
        }
        for(auto node : p_base->getSkeletonPath()){
            marker.id = marker_id;
            if(marker_id > max_marker_id){
                marker.action = visualization_msgs::Marker::ADD;
                max_marker_id = marker_id;
            } else{
                marker.action = visualization_msgs::Marker::MODIFY;
            }

            marker.color.a = 1;
            marker.color.r = 1;
            marker.color.g = 0;
            marker.color.b = 0;
            marker.pose.position.x = node.x;
            marker.pose.position.y = node.y;
            marker.pose.position.z = (car_z_min + car_z_max)/2;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            corridorSeq.markers.emplace_back(marker);
            marker_id++;
        }
        {
            Corridor search_range = p_base->getSearchRange();

            marker.id = marker_id;
            if(marker_id > max_marker_id){
                marker.action = visualization_msgs::Marker::ADD;
                max_marker_id = marker_id;
            } else{
                marker.action = visualization_msgs::Marker::MODIFY;
            }

            marker.color.a = 1;
            marker.color.r = 0;
            marker.color.g = 1;
            marker.color.b = 1;
//            marker.pose.position.x = (search_range.xu + search_range.xl) / 2;
//            marker.pose.position.y = (search_range.yu + search_range.yl) / 2;
//            marker.pose.position.z = (car_z_min + car_z_max)/2;
//            marker.scale.x = search_range.xu - search_range.xl;
//            marker.scale.y = search_range.yu - search_range.yl;
//            marker.scale.z = car_z_max - car_z_min;

            marker.type = visualization_msgs::Marker::LINE_LIST;

            marker.pose.position.x = 0;
            marker.pose.position.y = 0;
            marker.pose.position.z = 0;

            marker.pose.orientation.x = 0;
            marker.pose.orientation.y = 0;
            marker.pose.orientation.z = 0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.1;
            geometry_msgs::Point point;
            point.z = (car_z_min + car_z_max)/2;

            point.x = search_range.xl;
            point.y = search_range.yl;
            marker.points.emplace_back(point);
            point.x = search_range.xl;
            point.y = search_range.yu;
            marker.points.emplace_back(point);

            point.x = search_range.xl;
            point.y = search_range.yu;
            marker.points.emplace_back(point);
            point.x = search_range.xu;
            point.y = search_range.yu;
            marker.points.emplace_back(point);

            point.x = search_range.xu;
            point.y = search_range.yu;
            marker.points.emplace_back(point);
            point.x = search_range.xu;
            point.y = search_range.yl;
            marker.points.emplace_back(point);

            point.x = search_range.xu;
            point.y = search_range.yl;
            marker.points.emplace_back(point);
            point.x = search_range.xl;
            point.y = search_range.yl;
            marker.points.emplace_back(point);

            corridorSeq.markers.emplace_back(marker);
            marker_id++;
        }

        for(int i = marker_id; i <= max_marker_id; i++){
            marker.id = i;
            marker.action = visualization_msgs::Marker::DELETE;
            corridorSeq.markers.emplace_back(marker);
        }
        max_marker_id = marker_id - 1;

        mSet[1].unlock();
    }else{
//        ROS_WARN("[RosWrapper] Locking failed for ros data update. The output of p_base is being modified in planner ");
    }

    // 2. topics which is not obtained from planning thread
    // TODO multiple obstacles
    pubObservationMarker.publish(p_base->predictorSet[0].get_obsrv_marker(worldFrameId));
    // prepare the obstacle prediction info

    obstaclePrediction.markers.clear();

    for(auto obstPath : p_base->getCurObstaclePathArray().obstPathArray){
        // per a obstacle path, make up marker array
        // TODO shope should be rigorously considered
        visualization_msgs::Marker m_obstacle_rad;
        m_obstacle_rad.header.frame_id = worldFrameId;
        m_obstacle_rad.pose.orientation.w = 1.0;
        m_obstacle_rad.color.r = 1.0, m_obstacle_rad.color.a = 0.8;
        m_obstacle_rad.type = 3;
        int id = 0 ;
        for (auto pnt : obstPath.obstPath){
            m_obstacle_rad.id = id++;
            m_obstacle_rad.pose.position.x = pnt.q(0);
            m_obstacle_rad.pose.position.y = pnt.q(1);
            m_obstacle_rad.pose.position.z = p_base->predictorSet[0].getHeight();
            m_obstacle_rad.scale.x = 2*pow(pnt.Q(0,0),-1/2.0);
            m_obstacle_rad.scale.y = 2*pow(pnt.Q(1,1),-1/2.0);
            m_obstacle_rad.scale.z = 0.3;
            obstaclePrediction.markers.push_back(m_obstacle_rad);
        }
    }

    pubPredictionArray.publish(obstaclePrediction);


}

/**
 * @brief publish the car input and visualization markers
 * @details Do not use p_base here !!
 */
void RosWrapper::publish() {
    // e.g pub1.publish(topic1)
    if (p_base->isLPsolved)
        pubCurCmd.publish(p_base->getCurInput(curTime()));
    pubPath.publish(planningPath);
    pubCorridorSeq.publish(corridorSeq);
}

/**
 * @brief While loop of the ros wrapper (publish and spin)
 */
void RosWrapper::runROS() {
        ros::Rate lr(50);

        while(ros::ok()){
            updatePrediction();
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
        // make xy
        curState.x = dataPtr->pose.position.x;
        curState.y = dataPtr->pose.position.y;

        // make theta
        tf::Quaternion q;
        q.setX(dataPtr->pose.orientation.x);
        q.setY(dataPtr->pose.orientation.y);
        q.setZ(dataPtr->pose.orientation.z);
        q.setW(dataPtr->pose.orientation.w);

        tf::Transform Twc; Twc.setRotation(q);
        tf::Matrix3x3 Rwc = Twc.getBasis();
        tf::Vector3 e1 = Rwc.getColumn(0);
        double theta = atan2(e1.y(),e1.x());
        curState.theta = theta;

        // make v
        curState.v = speed; // reverse gear = negative

        ROS_DEBUG("Current car state (x,y,theta(degree),v) : [%f,%f,%f,%f]",curState.x,curState.y,curState.theta*180/M_PI,curState.v);

        p_base->setCarState(curState);
        mSet[0].unlock();
        isCarPoseCovReceived = true;
//        ROS_INFO("[RosWrapper] car pose update");
    }else{
//        ROS_WARN("[RosWrapper] callback for CarPoseCov locked by planner. Passing update");
    }
}

void RosWrapper::cbCarSpeed(const std_msgs::Float64 speed_) {
    speed = speed_.data;
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
//        ROS_WARN("[RosWrapper] callback for CarPoseCov locked by planner. Passing update");
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
//        ROS_WARN("[RosWrapper] callback for CarPoseCov locked by planner. Passing update");
    }
}

/**
 * @brief receive the global map and update
 * @param octomap_msg
 */
void RosWrapper::cbLocalMap(const octomap_msgs::Octomap& octomap_msg) {
    // TODO you have to decide whether the update in this callback could interrupt planning thread
    if(mSet[0].try_lock()){
        p_base->setLocalMap(dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(octomap_msg)));
        mSet[0].unlock();
        isLocalMapReceived = true;
//        ROS_INFO("[RosWrapper] local map update");
    }else{
//        ROS_WARN("[RosWrapper] callback for CarPoseCov locked by planner. Passing update");
    }
}

/**
 * @brief update the observation for obstacles
 * @param obstPose
 * @todo we might have to include the geometry shape in the future...
 */
void RosWrapper::cbObstacles(const geometry_msgs::PoseStamped& obstPose) {

    ROS_INFO_ONCE("[SNU_PLANNER/RosWrapper] got first obstacle observation");
    double t = curTime();
    p_base->predictorSet[0].update_observation(t,obstPose.pose.position);

}

/**
 * @brief Whether all the necessary inputs are received
 * @return true if all the necessary inputs are received
 */
bool RosWrapper::isAllInputReceived() {
    return
//              isGlobalMapReceived and
              isLocalMapReceived and
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
    Predictor::TargetManager predictor(param.p_param.queueSize,param.p_param.zHeight,param.p_param.polyOrder);
    p_base_shared->predictorSet.push_back(predictor);

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

bool Wrapper::plan(double tTrigger){
    mSet[0].lock();
//    ROS_INFO( "[Wrapper] Assume that planning takes 0.5 sec. Locking subscription.\n ");
//    std::this_thread::sleep_for(std::chrono::duration<double>(0.5));

    // call global planner
    bool gpPassed = gp_ptr->plan(tTrigger);
    if (not p_base_shared->isGPsolved)
        p_base_shared->isGPsolved = gpPassed;

    if (gpPassed) {
        ROS_INFO_ONCE("[SNU_PLANNER/Wrapper] global planner passed. Start local planning ");
        updateCorrToBase();

        // Update p_base. the prediction model is being updated in ROS Wrapper
        int nStep  = param.l_param.horizon/param.l_param.tStep; // the division should be integer
        VectorXd tSeq(nStep);
        tSeq.setLinSpaced(nStep,tTrigger,tTrigger+param.l_param.horizon);
        p_base_shared->updatePrediction(tSeq,param.l_param.obstRadiusNominal);


        // Call local planner
        bool lpPassed =false ;
        lpPassed = lp_ptr->plan(tTrigger); // TODO

        if (lpPassed)
            //cout<<"Okay, fine"<<endl;
            updateMPCToBase();
        else
            ROS_WARN("[SNU_PLANNER/Wrapper] local planning failed");

        if (not p_base_shared->isLPsolved)
            p_base_shared->isLPsolved = lpPassed;

        // Unlocking
        mSet[0].unlock();
        return (gpPassed and lpPassed);

    }else{ //
        mSet[0].unlock();
        ROS_WARN("[SNU_PLANNER/Wrapper] global planning failed");
        return false;
    }
//    printf("----------------------------------------------------------------\n");


}

/**
 * @brief Modify p_base with the resultant planning output
 */
void Wrapper::updateCorrToBase() {
    mSet[1].lock();
//    ROS_INFO( "[Wrapper] Assume that updating takes 0.2 sec. Locking rosmsg update.\n ");
//    std::this_thread::sleep_for(std::chrono::duration<double>(0.2));
    gp_ptr->updateCorridorToBase();
    mSet[1].unlock();
//    ROS_INFO( "[Wrapper] p_base updated. Unlocking.\n ");
}
void Wrapper::updateMPCToBase() {
    mSet[1].lock();
//    ROS_INFO( "[Wrapper] Assume that updating takes 0.2 sec. Locking rosmsg update.\n ");
//    std::this_thread::sleep_for(std::chrono::duration<double>(0.2));
    lp_ptr->updateTrajToBase();
    mSet[1].unlock();
//    ROS_INFO( "[Wrapper] p_base updated. Unlocking.\n ");


}

/**
 * @brief Engage the while loop of planners
 */
void Wrapper::runPlanning() {

//    ROS_INFO( "[Wrapper] Assuming planning is updated at every 0.5 sec.\n"); // TODO
    // initial stuffs
    double Tp = 0.1; // sec
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
//                printf("================================================================");
//                ROS_INFO( "[Wrapper] Planning started."); // TODO

                // Do planning

                ROS_DEBUG("[Wrapper] goal : [%f,%f] / cur position : [%f,%f]",
                          p_base_shared->getDesiredState().x,p_base_shared->getDesiredState().y,
                        p_base_shared->getCarState().x,p_base_shared->getCarState().y);

                // prepare prediction sequence for  MPC. for time window (tcur,tcur+horizon)
                isPlanSuccess = plan(ros_wrapper_ptr->curTime());
//                printf("================================================================");

                ROS_INFO_STREAM("[Wrapper] planning time: " << std::chrono::duration_cast<std::chrono::microseconds>(chrono::steady_clock::now() - tCkp).count()*0.001 << "ms");
                // Only when the planning results are valid, we update p_base
                // At this step, the prepareROSmsgs() of RosWraper is unavailable
                if (isPlanSuccess){
                }
                else{
                    ROS_ERROR("[Wrapper] planning failed");
                }
            }
            // TODO update the obstaclePathArray to monitor current feasibilty
            // Trigger condition of planning. This can be anything other than the simple periodic triggering
            doPlan = chrono::steady_clock::now() - tCkp > std::chrono::duration<double>(Tp);
            ros::Rate(30).sleep();
        }else{
            ROS_WARN_ONCE("[Wrapper] waiting planning input subscriptions..");
        }
    }
}




