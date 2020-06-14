//
// Created by jbs on 20. 4. 6..
//

#include <atypical_planner/Wrapper.h>

using namespace Planner;


/////////////////
// Ros Wrapper //
/////////////////

bool Planner::comparePredictorId(const Predictor::IndexedPredictor &p1, int id ) {

    return get<0>(p1) == id;

}

/**
 * Constructor for ros wrapper
 * @param p_base_
 * @param mSet_ mutex set of size 2.
 */
RosWrapper::RosWrapper(shared_ptr<PlannerBase> p_base_):p_base(p_base_),nh("~"){
	cout << "-------------------------------------------------------" <<endl;
    // Load lanemap from parser
    string csv_file; double laneWidth;
    nh.param<string>("lane_csv_file",csv_file,"catkin_ws/src/atypical_driving_snu/keti_pangyo_path3.csv");
    nh.param<double>("lane_width",laneWidth,2.5);
    nh.param<string>("log_file_prefix",p_base_->log_file_name_base,"");

    // Logger reset
    string corridor_logger = p_base_->log_file_name_base;
    string mpc_logger = p_base->log_file_name_base;
    string state_logger = p_base->log_file_name_base;
    string input_logger = p_base->log_file_name_base;

    corridor_logger += "_corridor.txt";
    mpc_logger += "_mpc.txt";
    state_logger += "_state.txt";
    input_logger += "_input.txt";

    remove(corridor_logger.c_str());
    remove(mpc_logger.c_str());
    remove(state_logger.c_str());
    remove(input_logger.c_str());

    // Get lane information
    p_base->parse_tool.get_Coorddata(csv_file); // This parse only the center information

//     p_base->parse_tool.display_result();
    p_base->lane_path = (p_base->parse_tool.get_lanepath());
    // TODO (accurate lane width)
    p_base->lane_path.setWidth(laneWidth);

    isLaneReceived =false; // Still false. After applying Tw0, it is true
    isLaneRawReceived = true;

    // Initiate ros communication (caution: parameters are parsed in udpateParam)
    max_marker_id = 0;

    // Publisher
    pubPath = nh.advertise<nav_msgs::Path>("planning_path",1);
    pubCorridorSeq = nh.advertise<visualization_msgs::MarkerArray>("corridor_seq",1);
    pubObservationMarker = nh.advertise<visualization_msgs::MarkerArray>("observation_queue",1);
    pubPredictionArray = nh.advertise<visualization_msgs::MarkerArray>("prediction",1);
    pubCurCmd = nh.advertise<driving_msgs::VehicleCmd>("/vehicle_cmd",1);
    pubMPCTraj = nh.advertise<nav_msgs::Path>("mpc_traj",1);
    pubCurPose = nh.advertise<geometry_msgs::PoseStamped>("cur_pose",1);

    pubOrigLane = nh.advertise<nav_msgs::Path>("lane_orig",1);
    pubSlicedLane = nh.advertise<nav_msgs::Path>("lane_sliced",1);
    pubSideLane = nh.advertise<visualization_msgs::MarkerArray>("side_lanes",1);
    pubCurGoal = nh.advertise<geometry_msgs::PointStamped>("global_goal",1);

    pubSmoothLane = nh.advertise<visualization_msgs::MarkerArray>("/smooth_lane",1);
    pubTextSlider = nh.advertise<visualization_msgs::Marker>("current_lane",1);

    // Subscriber
    subCarPoseCov = nh.subscribe("/current_pose",1,&RosWrapper::cbCarPoseCov,this);
    subCarSpeed = nh.subscribe("/current_speed",1,&RosWrapper::cbCarSpeed,this);
    //subExampleObstaclePose = nh.subscribe("obstacle_pose",1,&RosWrapper::cbObstacles,this);
    subDetectedObjects= nh.subscribe("/detected_objects",1,&RosWrapper::cbDetectedObjects,this);
    subOccuMap = nh.subscribe("/costmap_node/costmap/costmap",1,&RosWrapper::cbOccuMap,this); //TODO: fix /costmap_node/costmap/costmap to /occupancy_grid
}
/**
 * @brief update the fitting model only. It does not directly update the obstaclePath in p_base
 */
void RosWrapper::updatePredictionModel() {

    // 1. Update the fitting model
    for (auto it = p_base->indexedPredictorSet.begin();
            it !=  p_base->indexedPredictorSet.end();it++){
        // First, check whether the observation has expired
        double tCur = curTime();
        double tLast = get<1>(*it).getLastObservationTime();
        double tExpire = get<1>(*it).getExpiration();
        if( tCur - tLast  > tExpire) {
            ROS_INFO("[SNU_PLANNER/RosWrapper] predictor of %d has been destroyed.",get<0>(*it));
            p_base->indexedPredictorSet.erase(it++);
            continue;
        }
        // Next, if the predictor was not detached, we continue to update
        get<1>(*it).update_predict();

    }

    // 2. Upload the obstacle prediction over horizon

    // Update p_base. the prediction model is being updated in ROS Wrapper
    int nStep  = param.l_param.horizon/param.l_param.tStep; // the division should be integer
    VectorXd tSeq(nStep);
    tSeq.setLinSpaced(nStep,curTime(),curTime()+param.l_param.horizon);
    p_base->uploadPrediction(tSeq, param.l_param.obstRadiusNominal);
}


/**
 * @brief Fetching the parameter from ros node handle.
 * @param param_ The fetched params will be written in param_
 */
void RosWrapper::updateParam(Param &param_) {
    // Do some parsing here from reading from ros launch
    ROS_INFO("Reading the parameters from launch..");

    nh.param<string>("world_frame_id",worldFrameId,"/map");
    nh.param<string>("snu_frame_id",SNUFrameId,"/SNU");
    nh.param<string>("occu_map_frame_id",octomapGenFrameId,"/map");
    nh.param<string>("base_link_id",baseLinkId,"/base_link");
    nh.param<string>("detected_objects_id",detectedObjectId,"/map");




    // global planner

    nh.param<double>("vmax",param_.g_param.car_speed_max,4);
    nh.param<double>("vmin",param_.g_param.car_speed_min,1);
    nh.param<double>("curve_thres",param_.g_param.curvature_thres,(3.141592/3.0));

    nh.param<double>("global_planner/horizon",param_.g_param.horizon,15);
    nh.param<double>("global_planner/period",param_.g_param.period,2);

    nh.param<double>("global_planner/car_width",param_.g_param.car_width,2);
    nh.param<double>("global_planner/car_z_min",param_.g_param.car_z_min,0.0);
    nh.param<double>("global_planner/car_z_max",param_.g_param.car_z_max,2.0);
    nh.param<double>("global_planner/car_acceleration",param_.g_param.car_acceleration,1.0);
    nh.param<double>("global_planner/world_x_min",param_.g_param.world_x_min,-10);
    nh.param<double>("global_planner/world_y_min",param_.g_param.world_y_min,-1);
    nh.param<double>("global_planner/world_x_max",param_.g_param.world_x_max,35);
    nh.param<double>("global_planner/world_y_max",param_.g_param.world_y_max,80);
    nh.param<double>("global_planner/grid_resolution",param_.g_param.grid_resolution,0.5);
    nh.param<double>("global_planner/box_resolution",param_.g_param.box_resolution,0.3);
    nh.param<double>("global_planner/box_max_size",param_.g_param.box_max_size,10);
    nh.param<bool>("global_planner/is_world_snu_frame",param_.g_param.is_world_box_snu_frame,false);
    nh.param<int>("global_planner/max_smoothing_iteration", param_.g_param.max_smoothing_iteration,5);
    nh.param<double>("global_planner/smoothing_margin", param_.g_param.smoothing_margin,0.5);
    nh.param<double>("global_planner/max_steering_angle", param_.g_param.max_steering_angle, M_PI/6);

    // local planner
    nh.param<double>("local_planner/horizon",param_.l_param.horizon,5);
    nh.param<double>("local_planner/period",param_.l_param.period,0.1);
    nh.param<double>("local_planner/ts",param_.l_param.tStep,0.1);
    nh.param<double>("local_planner/obstacle_radius_nominal",param_.l_param.obstRadiusNominal,0.3);
    nh.param<double>("local_planner/car_longtitude",param_.l_param.carLongtitude,2.7);
    nh.param<double>("local_planner/max_steer",param_.l_param.maxSteer,M_PI/30);
    nh.param<double>("local_planner/max_accel",param_.l_param.maxAccel,3);
    nh.param<double>("local_planner/min_accel",param_.l_param.minAccel,-1);
    nh.param<double>("local_planner/car_speed",param_.l_param.nominal_speed,2.0);


    Parameter ilqr_weight;
    param_.l_param.final_weight = ilqr_weight.setting.final_weight;
    param_.l_param.input_weight = ilqr_weight.setting.input_weight;
    param_.l_param.state_weight = ilqr_weight.setting.state_weight;


    bool isUseSimTimeMode = false;


    // predictor
    nh.param<int>("predictor/observation_queue",param_.p_param.queueSize,6);
    nh.param<float>("predictor/ref_height",param_.p_param.zHeight,1.0);
    nh.param<int>("predictor/poly_order",param_.p_param.polyOrder,1); // just fix 1
    nh.param<double>("predictor/tracking_expiration",param_.p_param.trackingTime,2.0); // just fix 1

    // Common
    nh.param<double>("goal_thres",param_.l_param.goalReachingThres,0.4); // just fix 1
    p_base->goal_thres = param_.l_param.goalReachingThres;

    nh.param<bool>("use_nominal_obstacle_rad",use_nominal_obstacle_radius,false); // just fix 1
    if(use_nominal_obstacle_radius)
        ROS_INFO("[SNU_PLANNER/RosWrapper] We assume fixed-size obstacle.");


    while (ros::Time::now().toSec()== 0 ){
        ROS_WARN("[SNU_PLANNER/RosWrapper] current ros time is zero. is running in sim time mode? Idling until valid ros clock");
        ros::Rate(20).sleep();
    }
    ROS_INFO("[SNU_PLANNER/RosWrapper] Initialized clock with ROS time %f",ros::Time::now().toSec());
    t0 = ros::Time::now().toSec();


    vector<double> target_waypoint_x;
    vector<double> target_waypoint_y;

    bool hasSeqGoal = nh.getParam("target_waypoint/x",target_waypoint_x);
    nh.getParam("target_waypoint/y",target_waypoint_y);
    if (hasSeqGoal) {
        for (int i = 0; i < target_waypoint_x.size(); i++) {
            ROS_INFO("[SNU_PLANNER/RosWrapper] received global goal [%f,%f] (in global frame)",
                     target_waypoint_x[i], target_waypoint_y[i]);
            p_base->desired_state_seq.push_back(CarState{target_waypoint_x[i],target_waypoint_y[i],0,0});
        }
    }
    else{
        ROS_ERROR("[SNU_PLANNER/RosWrapper] no goal sequence given");
        return;
    }

    param = param_;
}

/**
 * @breif Convert the row information of p_base into rosmsgs
 * @details Extract information from p_base
 */
void RosWrapper::prepareROSmsgs() {

    // 1. Topics directly obtained from p_base

    // planning path
    planningPath.poses.clear();
    geometry_msgs::PoseStamped poseStamped;

    /**
     * MUTEX
     */
    p_base->mSet[1].lock();
    for (auto pose : p_base->getMPCResultTraj().xs){
        poseStamped.pose.position.x = pose.x;
        poseStamped.pose.position.y = pose.y;
        planningPath.poses.push_back(poseStamped);
        }
    p_base->mSet[1].unlock();

    // MPC path
    if (p_base->isLPsolved) {
        MPCTraj.header.frame_id = SNUFrameId;
        MPCTraj.poses.clear();
        PoseStamped MPCPose;

        p_base->mSet[1].lock();
        MPCResultTraj mpcResultTraj = p_base->getMPCResultTraj();
        p_base->mSet[1].unlock();

        double t = mpcResultTraj.ts[0];
        double dt = 0.1;
        while(t < mpcResultTraj.ts[mpcResultTraj.ts.size()-1]){
            CarState state = mpcResultTraj.evalX(t);
            MPCPose.pose.position.x = state.x;
            MPCPose.pose.position.y = state.y;
            MPCTraj.poses.emplace_back(MPCPose);
            t += dt;
        }
    }

    // Current goal
    geometry_msgs::PointStamped curGoal;
    curGoal.header.frame_id = SNUFrameId;
    curGoal.point.x = p_base->getDesiredState().x;
    curGoal.point.y = p_base->getDesiredState().y;
    pubCurGoal.publish(curGoal);
    p_base->cur_pose.header.frame_id = SNUFrameId;
    pubCurPose.publish(p_base->getCurPose());

    // Current lane information
    visualization_msgs::Marker laneInfoText;
    laneInfoText.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    laneInfoText.text = "curv: " + to_string(p_base->laneCurvature) + "/speed_nom: " + to_string(p_base->laneSpeed);
    laneInfoText.header.frame_id = baseLinkId;
    laneInfoText.pose.orientation.w = 1.0;
    laneInfoText.pose.position.x = -5;
    laneInfoText.pose.position.y = -5;
    laneInfoText.color.a = 1.0;
    laneInfoText.color.g = 1.0;
    laneInfoText.scale.z = 0.7;
    pubTextSlider.publish(laneInfoText);

    // 2. topics which is not obtained from planning thread
    visualization_msgs::MarkerArray observations;
    int nsId = 0;
    for(auto idPredictor : p_base->indexedPredictorSet){
        observations.markers.push_back(get<1>(idPredictor).get_obsrv_marker(SNUFrameId,nsId++));
    }

    ROS_DEBUG("Number of predictor = %d",p_base->indexedPredictorSet.size());
    ROS_DEBUG("last id  = %d",nsId);

    pubObservationMarker.publish(observations);

    obstaclePrediction.markers.clear();
    nsId = 0;
    for(auto obstPath : p_base->getCurObstaclePathArray().obstPathArray){
        // per a obstacle path, make up marker array
        // TODO shape should be rigorously considered
        visualization_msgs::Marker m_obstacle_rad;
        m_obstacle_rad.header.frame_id = SNUFrameId;
        m_obstacle_rad.pose.orientation.w = 1.0;
        m_obstacle_rad.color.r = 1.0, m_obstacle_rad.color.a = 0.8;
        m_obstacle_rad.type = 3;
        m_obstacle_rad.ns = to_string(nsId);
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
        nsId++;
    }
    pubPredictionArray.publish(obstaclePrediction); // <- why this is in here? It should go to publish() (jungwon)
}

/**
 * @brief publish the car input and visualization markers
 * @details Do not use p_base here !!
 */
void RosWrapper::publish() {

    // 1. Actuation command
    if (p_base->isLPsolved) {
        auto cmd = p_base->getCurInput(curTime());
        cmd.header.stamp = ros::Time::now();
        cmd.steer_angle_cmd *= (180.0/3.14); // cmd output deg
        pubCurCmd.publish(cmd);
        pubMPCTraj.publish(MPCTraj);
        p_base->log_state_input(curTime());;
    }
    pubPath.publish(planningPath);
    pubCorridorSeq.publish(corridorSeq);

    p_base->mSet[1].lock();
    if (isLaneReceived){
        pubOrigLane.publish(p_base->laneOrig.getPath(SNUFrameId));
        pubSideLane.publish(p_base->laneOrig.getSidePath(SNUFrameId));
    }
    if(isLaneSliceLoaded){
        pubSlicedLane.publish(p_base->laneSliced.getPath(SNUFrameId));
    }
    pubSmoothLane.publish(p_base->laneSmooth.getPoints(SNUFrameId));
    p_base->mSet[1].unlock();

}

/**
 * @brief While loop of the ros wrapper (publish and spin)
 */
void RosWrapper::runROS() {
        ros::Rate lr(50);

        while(ros::ok()){
            updatePredictionModel();
            prepareROSmsgs(); // you may trigger this only under some conditions
            publish();
            processTf();
            ros::spinOnce(); // callback functions are executed
            lr.sleep();
        }
}

/**
 * @brief broadcasting and listening tf
 */
void RosWrapper::processTf() {
    // Broadcasting


    // 2. Transform broadcasting the tf of the current car w.r.t the first received tf

    if (isFrameRefReceived and isCarPoseCovReceived) {
        // (a) send Tsb
        auto pose = p_base->getCurPose();

        tf::Transform transform;
        transform.setOrigin( tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) );
        tf::Quaternion q;
        q.setX(pose.pose.orientation.x);
        q.setY(pose.pose.orientation.y);
        q.setZ(pose.pose.orientation.z);
        q.setW(pose.pose.orientation.w);
        transform.setRotation(q);
        tf_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),SNUFrameId, baseLinkId));


        // (b) send Tws (static)
        transform.setOrigin(tf::Vector3(p_base->Tws.translation()(0),p_base->Tws.translation()(1),p_base->Tws.translation()(2)));
        auto qd = Eigen::Quaterniond(p_base->Tws.rotation());
        q.setX(qd.x());
        q.setY(qd.y());
        q.setZ(qd.z());
        q.setW(qd.w());
        transform.setRotation(q);//
        tf_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), worldFrameId, SNUFrameId));

        // (c) lookup frame occupancy referance frame
        try {

            tf::StampedTransform Tsr;// SNU frame to the referance frame of obstacle
            tf_ls.lookupTransform(SNUFrameId,octomapGenFrameId, ros::Time(0), Tsr);
            p_base->mSet[0].lock();
            p_base->Tso.setIdentity();
            p_base->Tso.translate(Vector3d(Tsr.getOrigin().x(),Tsr.getOrigin().y(),Tsr.getOrigin().z()));
            isOctomapFrameResolved = true;
            Eigen::Quaterniond q;
            q.x() = Tsr.getRotation().getX();
            q.y() = Tsr.getRotation().getY();
            q.z() = Tsr.getRotation().getZ();
            q.w() = Tsr.getRotation().getW();
            p_base->Tso.rotate(q);
            p_base->mSet[0].unlock();

        }
        catch(tf::TransformException ex){
            ROS_ERROR("[SNU_PLANNER/RosWrapper] No tf connecting SNU frame with header of occupancy");
        }
    }

}

void RosWrapper::cbDetectedObjects(const driving_msgs::DetectedObjectArray &objectsArray) {
    if (isCarPoseCovReceived)
        for(auto object : objectsArray.objects) {


            bool valueCheck = true;
            valueCheck = not(object.odom.pose.pose.position.x == 0 and object.odom.pose.pose.position.y == 0  );
            if (not valueCheck)
                continue;


            // Convert it into SNU frame

            string obstacleRefFrame = detectedObjectId;
            tf::StampedTransform Tsr;// SNU frame to the referance frame of obstacle

            try {
                tf_ls.lookupTransform(SNUFrameId, obstacleRefFrame, ros::Time(0), Tsr);
            }
            catch(tf::TransformException ex){
                ROS_ERROR("[SNU_PLANNER/RosWrapper] cannot register the object pose. "
                          "No tf connecting SNU frame with header of object");
            }

            tf::Vector3 pr(object.odom.pose.pose.position.x,object.odom.pose.pose.position.y,0); // translation w.r.t its ref frame
            tf::Vector3 ps = Tsr*pr; // w.r.t to SNU

            // TODO full pose to be implemented
            geometry_msgs::Point position;
            position.x = ps.x();
            position.y = ps.y();

            uint id = object.id;

            // Does this id have its predictor?
            auto predictorOwner = find_if(p_base->indexedPredictorSet.begin(),
                    p_base->indexedPredictorSet.end(),bind(comparePredictorId,placeholders::_1,id));
            // already a predictor owns the id
            Vector3f updateDimension;
            if (use_nominal_obstacle_radius){
                updateDimension.x() = param.l_param.obstRadiusNominal;
                updateDimension.y() = param.l_param.obstRadiusNominal;
                updateDimension.z() = param.l_param.obstRadiusNominal;
            }else{
                updateDimension.x() = object.dimensions.x;
                updateDimension.y() = object.dimensions.y;
                updateDimension.z() = object.dimensions.z;
            }

            if (predictorOwner != p_base->indexedPredictorSet.end()) {
                get<1>(*predictorOwner).update_observation(curTime(), position,
                        updateDimension);
            } else {
                // no owner found, we create predictor and attach it
                auto newPredictor = make_tuple(id, p_base->predictorBase);
                get<1>(newPredictor).update_observation(curTime(), position,
                        updateDimension);
                p_base->indexedPredictorSet.push_back(newPredictor);
                ROS_INFO("[SNU_PLANNER/RosWrapper] Predictor attached for obstacle id = %d", id);
            }
        }
    else
        ROS_INFO("[SNU_PLANNER/RosWrapper] Received object information. But the state of the car is not received");
}


/**
 * @brief receive the car pose Cov and update
 * @param dataPtr
 */
void RosWrapper::cbCarPoseCov(geometry_msgs::PoseWithCovarianceConstPtr dataPtr) {

    // First, generate the ref frame based on the first-received pose
    if (not isFrameRefReceived){
        if (dataPtr->pose.position.x == 0 and dataPtr->pose.position.y == 0 ){
            ROS_ERROR("[SNU_PLANNER/RosWrapper] The first received pose is just zero. It is ignored as the ref tf");
            return;
        }
        p_base->Tws.setIdentity(); // just initialization
        Eigen::Quaterniond quat;
        Eigen::Vector3d transl;

        transl(0) = dataPtr->pose.position.x;
        transl(1) = dataPtr->pose.position.y;
        transl(2) = dataPtr->pose.position.z;
        // due to jungwon
        quat.x() =  dataPtr->pose.orientation.x;
        quat.y() =  dataPtr->pose.orientation.y;
        quat.z() =  dataPtr->pose.orientation.z;
        quat.w() =  dataPtr->pose.orientation.w;

        p_base->Tws.translate(transl);
        p_base->Tws.rotate(quat);

        ROS_INFO("[SNU_PLANNER/RosWrapper] Reference tf has been initialized with [%f,%f,%f,%f,%f,%f,%f]",
                transl(0),transl(1),transl(2),quat.x(),quat.y(),quat.z(),quat.w());
        isFrameRefReceived = true;

        // If ref frame was set, then transform the laneNode
        if (isLaneRawReceived){
            auto lane_w = p_base->getLanePath(); // w.r.t world frame
            lane_w.applyTransform(p_base->Tws.inverse());
            p_base->setLanePath(lane_w); // w.r.t SNU frame
            ROS_INFO("[SNU_PLANNER/RosWrapper] Lane-path transform completed!");
            p_base->laneOrig = Lane(lane_w); // should be applied transform
            isLaneReceived = true;
        }else{
            ROS_ERROR("Tried transforming Lane raw in the pose callback. But no lane exsiting");
        }

        // Convert the global goal to local goal
        for (auto &goal  : p_base->desired_state_seq){
            Vector4d xb(goal.x,goal.y,0,1);
            Vector4d xa = p_base->Tws.inverse()*xb;
            goal.x = xa(0);
            goal.y = xa(1);
            ROS_INFO("[SNU_PLANNER/RosWrapper] received global goal [%f,%f] (in SNU frame)",goal.x,goal.y);
        }
        isGoalReceived = true;
    }
    // Retreive Tsb
    if (isCarSpeedReceived) {

        // 1. Converting car pose w.r.t Tw0
        auto poseOrig = dataPtr->pose;
        SE3 Tw1 = DAP::pose_to_transform_matrix(poseOrig).cast<double>(); // Tw1
        SE3 T01 = p_base->Tws.inverse() * Tw1;
        auto poseTransformed = DAP::transform_matrix_to_pose(T01.cast<float>());
        // Update the pose information w.r.t Tw1
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.frame_id = SNUFrameId;
        poseStamped.pose = poseTransformed;

        // 2. CarState
        CarState curState;

        // make xy
        curState.x = poseTransformed.position.x;
        curState.y = poseTransformed.position.y;

        // make theta
        tf::Quaternion q;
        q.setX(poseTransformed.orientation.x);
        q.setY(poseTransformed.orientation.y);
        q.setZ(poseTransformed.orientation.z);
        q.setW(poseTransformed.orientation.w);

        tf::Transform Twc;
        Twc.setIdentity();
        Twc.setRotation(q);

        tf::Matrix3x3 Rwc = Twc.getBasis();
        tf::Vector3 e1 = Rwc.getColumn(0);
        double theta = atan2(e1.y(), e1.x());
        curState.theta = theta;

        // make v
        curState.v = speed; // reverse gear = negative
        ROS_DEBUG("Current car state (x,y,theta(degree),v(m/s)) : [%f,%f,%f,%f]", curState.x, curState.y,
                  curState.theta * 180 / M_PI, curState.v);

        /**
         * MUTEX - upload
         */
         p_base->mSet[0].lock();
         p_base->Tsb = T01;
         p_base->setCurPose(poseStamped);
         p_base->setCarState(curState);
         p_base->mSet[0].unlock();

         isCarPoseCovReceived = true;
        ROS_INFO_ONCE("[SNU_PLANNER/RosWrapper] First received car state! ");
    }else {
        ROS_WARN("[SNU_PLANNER/RosWrapper] Car speed is not being received. CarState will not be updated");
    }
}

void RosWrapper::cbOccuMap(const nav_msgs::OccupancyGrid & occuMap) {

    p_base->localMap = occuMap;
    ROS_INFO_STREAM_ONCE("[SNU_PLANNER/RosWrapper] Received first occupancy map");
    isLocalMapReceived = true;
}

void RosWrapper::cbCarSpeed(const std_msgs::Float64& speed_) {
    speed = speed_.data*1000.0/3600;
    isCarSpeedReceived = true;
    p_base->cur_state.v = speed; // reverse gear = negative
}
/**
 * @brief update the observation for obstacles
 * @param obstPose
 * @todo we might have to include the geometry shape in the future...
 */
void RosWrapper::cbObstacles(const geometry_msgs::PoseStamped& obstPose) {

    ROS_INFO_ONCE("[SNU_PLANNER/RosWrapper] got first obstacle observation");
    double t = curTime();
    // p_base->predictorSet[0].update_observation(t,obstPose.pose.position); // Deprecated

}

/**
 * @brief Whether all the necessary inputs are received
 * @return true if all the necessary inputs are received
 */
bool RosWrapper::isAllInputReceived() {

    bool isOKKK = true;
    if (not isLocalMapReceived) {
        ROS_ERROR_THROTTLE(2,"[SNU_PLANNER/RosWrapper] Still no occupancy grid received.");
        return false;
    }
    if (not isCarPoseCovReceived) {
        ROS_ERROR_THROTTLE(2,"[SNU_PLANNER/RosWrapper] Still no car pose received.");
        return false;
    }
    if (not isCarSpeedReceived) {
        ROS_ERROR_THROTTLE(2,"[SNU_PLANNER/RosWrapper] Still no car speed received ");
        return false;
    }
    if (not isFrameRefReceived) {
        ROS_ERROR_THROTTLE(2,"[SNU_PLANNER/RosWrapper] Still no referance frame (SNU) fixed.");
        return false;
    }

    if (not isLaneReceived) {
        ROS_ERROR_THROTTLE(2,"[SNU_PLANNER/RosWrapper] No lane information loaded");
        return false;
    }
    if (not isOctomapFrameResolved){
        ROS_ERROR_THROTTLE(2,"[SNU_PLANNER/RosWrapper] Still tf from occupancy to SNU not resolved.");
        return false;
    }
    return true;
}



////////////////////////////////////
// Wrapper (top of the hierarchy) //
////////////////////////////////////


/**
 * @brief The constructor 1) initiates all the ros communication, 2) feed params to planners
 */
Wrapper::Wrapper() : p_base_shared(make_shared<PlannerBase>()) {

    // 1. Parse all parameters from nh
    ros_wrapper_ptr = new RosWrapper(p_base_shared);
    ros_wrapper_ptr->updateParam(param);

    // 2. initialize planners
    lp_ptr = new LocalPlannerPlain(param.l_param,p_base_shared); // TODO Stochastic MPC also available
    // cout << p_base_shared.use_count() << endl;
    gp_ptr = new GlobalPlanner(param.g_param,p_base_shared);
    // cout << p_base_shared.use_count() << endl;
    Predictor::TargetManager predictor(param.p_param.queueSize,param.p_param.zHeight,param.p_param.polyOrder);
    p_base_shared->predictorBase = predictor;
    p_base_shared->predictorBase.setExpiration(param.p_param.trackingTime);
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
 * @brief lane slicing + Jungwon
 * @param tTrigger
 * @todo Jungwon
 */
void Wrapper::processLane(double tTrigger) {

    if (ros_wrapper_ptr->isLocalMapReceived) {


        // Occupancy map frame = octomapGenFrameId
        auto curOccupancyGrid = p_base_shared->localMap;
        Vector2d windowOrig(curOccupancyGrid.info.origin.position.x, curOccupancyGrid.info.origin.position.y);
        double windowWidth = curOccupancyGrid.info.width * curOccupancyGrid.info.resolution;
        double windowHeight = curOccupancyGrid.info.height * curOccupancyGrid.info.resolution;

        Vector3d windowOrigSNU = p_base_shared -> Tso * Vector3d(windowOrig(0),windowOrig(1),0);

        ROS_INFO("orig window = [%f,%f]",windowOrigSNU(0),windowOrigSNU(1));
        CarState curCarState = p_base_shared->getCarState(); // SNU frame
        int idxSliceStart,idxSliceEnd;
        vector<Vector2d> pathSliced = p_base_shared->laneOrig.slicing(curCarState, Vector2d(windowOrigSNU(0),windowOrigSNU(1)), windowWidth,
                                                                      windowHeight,idxSliceStart,idxSliceEnd);
        double meanCurv = meanCurvature(pathSliced);
        double vLaneRef; // referance velocity for the current lane
        double vmin = param.g_param.car_speed_min;
        double vmax = param.g_param.car_speed_max;
        double rho_thres = param.g_param.curvature_thres;

        if (meanCurv > rho_thres)
            vLaneRef = vmin;
        else{
            vLaneRef = vmax - (vmax-vmin)/rho_thres*meanCurv;
        }


        p_base_shared->mSet[1].lock();
        p_base_shared->laneSliced.points = pathSliced;
        p_base_shared->laneSliced.widths = vector<double>(p_base_shared->laneOrig.widths.begin()+idxSliceStart,p_base_shared->laneOrig.widths.begin()+idxSliceEnd+1);
        p_base_shared->laneSpeed = vLaneRef;
        p_base_shared->laneCurvature = meanCurv;
        ROS_INFO("lane [%f,%f]" ,meanCurv,vLaneRef);
        p_base_shared->mSet[1].unlock();

        if (not ros_wrapper_ptr->isLaneSliceLoaded){
            ros_wrapper_ptr->isLaneSliceLoaded = true;
            ROS_INFO("[SNU_PLANNER] Starting lane slicing! ");
        }


    }else{
        ROS_WARN_THROTTLE(2,"[SNU_PLANNER/Wrapper] Lane cannot be processed. No occupancy map received");

    }

}

bool Wrapper::planGlobal(double tTrigger){
    // call global planner
    bool gpPassed = gp_ptr->plan(tTrigger);
    if (not p_base_shared->isGPsolved)
        p_base_shared->isGPsolved = gpPassed;

    // let's log
    if (gpPassed) {
        updateCorrToBase();
    }
//    p_base_shared->log_corridor(tTrigger);

    return gpPassed;
}

bool Wrapper::planLocal(double tTrigger) {

//
//    // call global planner
//    bool lpPassed = lp_ptr->plan(tTrigger); // TODO
//    // if (not p_base_shared->isGPsolved)
//    if (not p_base_shared->isLPsolved)
//        p_base_shared->isLPsolved = lpPassed;
//    // let's log
//    if (lpPassed) {
//        updateMPCToBase();
//    }
//    p_base_shared->log_corridor(tTrigger,tTrigger + param.l_param.horizon);
//    p_base_shared->log_mpc(tTrigger);
//    return lpPassed;
    return true;
}


/**
 * @brief Modify p_base with the resultant planning output
 */
void Wrapper::updateCorrToBase() {
//    ROS_INFO( "[Wrapper] Assume that updating takes 0.2 sec. Locking rosmsg update.\n ");
//    std::this_thread::sleep_for(std::chrono::duration<double>(0.2));
    gp_ptr->updateCorridorToBase();
//    ROS_INFO( "[Wrapper] p_base updated. Unlocking.\n ");
}
void Wrapper::updateMPCToBase() {
//    ROS_INFO( "[Wrapper] Assume that updating takes 0.2 sec. Locking rosmsg update.\n ");
//    std::this_thread::sleep_for(std::chrono::duration<double>(0.2));
    lp_ptr->updateTrajToBase();
//    ROS_INFO( "[Wrapper] p_base updated. Unlocking.\n ");
}

/**
 * @brief Engage the while loop of planners
 */
void Wrapper::runPlanning() {

//    ROS_INFO( "[Wrapper] Assuming planning is updated at every 0.5 sec.\n"); // TODO
    // initial stuffs
    double Tp = 4.0; // sec
    auto tCkpG = chrono::steady_clock::now(); // check point time
    auto tCkpL = chrono::steady_clock::now(); // check point time

    bool doGPlan = true; // turn on if we have started the class
    bool doLPlan = true; // turn on if we have started the class
    bool didLplanByG = false; // Lp already done in GP loop
    bool isPlanPossible = false;
    bool isGPSuccess = false;
    bool isLPSuccess = false;
    bool isAllGoalReach = false;

    while (ros::ok()){
        isAllGoalReach = p_base_shared->desired_state_seq.empty();
        if (isAllGoalReach){ // all goal was acheived
            ROS_INFO_ONCE("[Wrapper] All goal reched!");
            // idling (TODO)
        }
        else {
            // 1. monitor states
            ROS_DEBUG("[Wrapper] goal : [%f,%f] / cur position : [%f,%f]",
                      p_base_shared->getDesiredState().x, p_base_shared->getDesiredState().y,
                      p_base_shared->getCarState().x, p_base_shared->getCarState().y);



            // 2. Check the condition for planning
            isPlanPossible = ros_wrapper_ptr->isAllInputReceived();
            didLplanByG = false;

            if (isPlanPossible) {
                ROS_INFO_ONCE("[Wrapper] start planning!");
                // 3. Do planning (GP->LP) or LP only
                if (doGPlan) { // G plan
                    tCkpG = chrono::steady_clock::now(); // check point time
                    ROS_INFO("[Wrapper] begin GP..");

                    // Lane processing
                    processLane(ros_wrapper_ptr->curTime());


                    // Do planning
                    isGPSuccess = planGlobal(ros_wrapper_ptr->curTime());
                    if (isGPSuccess) { // let's call LP
                        ROS_INFO_STREAM("[Wrapper] GP success! planning time for gp: " <<
                                                                                       std::chrono::duration_cast<std::chrono::microseconds>(
                                                                                               chrono::steady_clock::now() -
                                                                                               tCkpG).count() * 0.001
                                                                                       << "ms");
                        ROS_INFO("[Wrapper] begin LP..");

                        auto tCkp_mpc = chrono::steady_clock::now();
                        isLPSuccess = planLocal(ros_wrapper_ptr->curTime()); // TODO time ?
                        didLplanByG = true;
                        if (isLPSuccess)
                            ROS_INFO_STREAM("[Wrapper] LP success! planning time for lp: " <<
                                                                                           std::chrono::duration_cast<std::chrono::microseconds>(
                                                                                                   chrono::steady_clock::now() -
                                                                                                   tCkp_mpc).count() *
                                                                                           0.001
                                                                                           << "ms");
                        else
                            ROS_INFO_STREAM("[Wrapper] LP failed.");
                    } else
                        ROS_INFO_STREAM("[Wrapper] GP failed.");
                } // G plan ended

                // L plan
                if (doLPlan and (not didLplanByG) and
                    p_base_shared->isGPsolved) { // trigger at every LP period only if it was not called in GP loop
                    tCkpL = chrono::steady_clock::now(); // check point time
                    auto tCkp_mpc = chrono::steady_clock::now();
                    ROS_INFO("[Wrapper] begin LP..");
                    isLPSuccess = planLocal(ros_wrapper_ptr->curTime()); // TODO time ?
                    didLplanByG = true;
                    if (isLPSuccess)
                        ROS_INFO_STREAM("[Wrapper] LP success! planning time for lp: " <<
                                                                                       std::chrono::duration_cast<std::chrono::microseconds>(
                                                                                               chrono::steady_clock::now() -
                                                                                               tCkp_mpc).count() * 0.001
                                                                                       << "ms");
                    else
                        ROS_INFO_STREAM("[Wrapper] LP failed.");
                } // L plan ended

                doGPlan = (chrono::steady_clock::now() - tCkpG > std::chrono::duration<double>(param.g_param.period));
                doLPlan = (chrono::steady_clock::now() - tCkpL > std::chrono::duration<double>(param.l_param.period));

            } else { // planning cannot be started
                ROS_WARN_THROTTLE(2,
                                  "[Wrapper] waiting planning input subscriptions.. (message print out every 2 sec)");
            }
        }
        if (p_base_shared->isGoalReach()) {
            ROS_INFO("[Wrapper] goal : [%f,%f] reached",
                      p_base_shared->getDesiredState().x, p_base_shared->getDesiredState().y);
            p_base_shared->desired_state_seq.pop_front();
        }
        ros::Rate(30).sleep();
    }
}




