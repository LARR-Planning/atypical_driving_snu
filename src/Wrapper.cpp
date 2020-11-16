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
    nh.param("smoothing_type", p_base->smoothing_type,0);
    nh.param("stopSpeed",p_base->stopSpeed,0.0);
    nh.param("moving_horizon",p_base->smooth_horizon, 4);
    nh.param("smooth_weight",p_base->weight_smooth, 1.0);
    nh.param("ignore_angle", p_base->ignore_angle,0.017);
    cout <<"sm" << p_base->weight_smooth << endl;
    nh.param("goal/x",p_base->goal_x,0.0);
    nh.param("goal/y",p_base->goal_y,0.0);

    ROS_INFO("[RosWrapper] final goal in world frame = [%f, %f]",p_base->goal_x,p_base->goal_y );


    processedPclPtr  = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>() );
    groundPclPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>() );


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

     p_base->parse_tool.display_result();
    p_base->lane_path = (p_base->parse_tool.get_lanepath());
    p_base->lane_path.setWidth(laneWidth);

    isLaneReceived =false; // Still false. After applying Tw0, it is true
    isLaneRawReceived = true;

    // Initiate ros communication (caution: parameters are parsed in udpateParam)
    max_marker_id = 0;
    count_corridors = 0;

    // Publisher
    pubLastPublishedInput = nh.advertise<geometry_msgs::TwistStamped>("publishedInput",1);
    pubPath = nh.advertise<nav_msgs::Path>("planning_path",1);
    pubCorridorSeq = nh.advertise<visualization_msgs::MarkerArray>("corridor_seq",1);
    pubObservationMarker = nh.advertise<visualization_msgs::MarkerArray>("observation_queue",1);
    pubObservationPoseArray = nh.advertise<geometry_msgs::PoseArray>("observation_pose_queue",1);
    pubPredictionArray = nh.advertise<visualization_msgs::MarkerArray>("prediction",1);
    pubCurCmd = nh.advertise<driving_msgs::VehicleCmd>("/vehicle_cmd",1);
//    pubCurCmdDabin = nh.advertise<geometry_msgs::Twist>("/acc_cmd",1);
    pubMPCTraj = nh.advertise<nav_msgs::Path>("mpc_traj",1);
    pubMPCTrajMarker = nh.advertise<visualization_msgs::MarkerArray>("mpc_traj_marker",1);
    pubCurPose = nh.advertise<geometry_msgs::PoseStamped>("cur_pose",1);
    pubPitching= nh.advertise<std_msgs::Float64>("imu_pitcing",1);

    pubOrigLane = nh.advertise<nav_msgs::Path>("lane_orig",1);
    pubSlicedLane = nh.advertise<nav_msgs::Path>("lane_sliced",1);
    pubSideLane = nh.advertise<visualization_msgs::MarkerArray>("side_lanes",1);
    pubCurGoal = nh.advertise<geometry_msgs::PointStamped>("global_goal",1);

    pubOurOccu = nh.advertise<nav_msgs::OccupancyGrid>("map_for_planning",1);
    pubFilteredPcl = nh.advertise<sensor_msgs::PointCloud2>("pcl_filtered",1);
    pubGroundPcl = nh.advertise<sensor_msgs::PointCloud2>("pcl_ground",1);


    pubSmoothLane = nh.advertise<visualization_msgs::MarkerArray>("/smooth_lane",1);
    pubTextTrackingVelocity = nh.advertise<visualization_msgs::MarkerArray>("/detected_objects_info",1);
    pubTextSlider = nh.advertise<visualization_msgs::Marker>("current_lane",1);
    pubDetectedObjectsPoseArray = nh.advertise<geometry_msgs::PoseArray>("detected_objects_prediction",1);

    pubNominalVelocity = nh.advertise<std_msgs::Float64>("/nominal_speed",1);
    pubCurCmdSteer = nh.advertise<std_msgs::Float64>("/vehicle_cmd_steer",1);
    pubCurCmdAcc = nh.advertise<std_msgs::Float64>("/vehicle_cmd_acc",1);


    // Subscriber
    subCarPoseCov = nh.subscribe("/current_pose",1,&RosWrapper::cbCarPoseCov,this);
    subCarSpeed = nh.subscribe("/current_speed",1,&RosWrapper::cbCarSpeed,this);
    subPcl = nh.subscribe("/velodyne_points",1,&RosWrapper::pclCallback,this);
    subKetiImu = nh.subscribe("/imu",1,&RosWrapper::cbImu,this);
    //subExampleObstaclePose = nh.subscribe("obstacle_pose",1,&RosWrapper::cbObstacles,this);
    subDetectedObjects= nh.subscribe("/detected_objects",1,&RosWrapper::cbDetectedObjects,this);
    subOccuMap = nh.subscribe("/costmap_node/costmap/costmap",100,&RosWrapper::cbOccuMap,this); //TODO: fix /costmap_node/costmap/costmap to /occupancy_grid
    subOccuUpdate = nh.subscribe("/costmap_node/costmap/costmap_updates",100,&RosWrapper::cbOccuUpdate,this); //TODO: fix /costmap_node/costmap/costmap to /occupancy_grid
}
/**
 * @brief update the fitting model and the obstacle path together
 */
void RosWrapper::updatePredictionModel() {


    p_base->mSet[0].lock();
    // 1. Update the fitting model
    for (list<Predictor::IndexedPredictor>::iterator it = p_base->indexedPredictorSet.begin();
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
    p_base->mSet[0].unlock();


//    cout <<"current predictor size = " << p_base->indexedPredictorSet.size() << endl;
    // 1. Update the fitting model
    // 2. Upload the obstacle prediction over horizon
    // Update p_base. the prediction model is being updated in ROS Wrapper
    int nStep  = param.l_param.horizon/param.l_param.tStep; // the division should be integer
    VectorXd tSeq(nStep);
    tSeq.setLinSpaced(nStep,curTime(),curTime()+param.l_param.horizon);
    // Upload the obstaclePath corresponding to the time
    if (use_nominal_obstacle_radius)
        p_base->uploadPrediction(tSeq, param.l_param.obstRadiusNominal);
    else
        p_base->uploadPrediction(tSeq);
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
    nh.param<string>("car_imu_frame_id",carImuFrameId,"/car_imu");
    nh.param<string>("detected_objects_id",detectedObjectId,"/map");

    // global planner
    nh.param<double>("vmax",param_.g_param.car_speed_max,4);
    nh.param<double>("v_ref_past_weight",param_.g_param.v_ref_past_weight,0.3);
    nh.param<double>("vmin",param_.g_param.car_speed_min,1);
    nh.param<double>("curve_thres",param_.g_param.curvature_thres,(3.141592/3.0));
    nh.param<double>("map/pcl_lx",param_.g_param.pcl_lx,40);
    nh.param<double>("map/pcl_ly",param_.g_param.pcl_ly,40);
    nh.param<double>("map/pcl_z_min",param_.g_param.pcl_z_min,-4);
    nh.param<double>("map/pcl_z_max",param_.g_param.pcl_z_max,3);
    nh.param<int>("map/pcl_dbscan_minpnts",param_.g_param.pcl_dbscan_minpnts,3);
    nh.param<double >("map/pcl_dbscan_eps",param_.g_param.pcl_dbscan_eps,0.1);
    nh.param<double >("map/ransac_post_inclusion_offset",param_.g_param.ransac_ground_offest,0.3);
    nh.param<double >("map/ransac_distance_threshold",param_.g_param.ransac_distance_threshold,0.2);
    nh.param<bool>("map/use_ransac",param_.g_param.use_ransac,true);

    nh.param<double>("global_planner/car_width",param_.g_param.car_width,2);
    nh.param<double>("global_planner/period",param_.g_param.period,2);
    nh.param<double>("global_planner/grid_resolution",param_.g_param.grid_resolution,0.3);
    nh.param<double>("global_planner/smoothing_distance",param_.g_param.smoothing_distance, 8);
    nh.param<double>("global_planner/smoothing_cliff_min_bias",param_.g_param.smoothing_cliff_min_bias, 1.0);
    nh.param<double>("global_planner/smoothing_cliff_ratio", param_.g_param.smoothing_cliff_ratio, 0.5);
    nh.param<double>("global_planner/max_steering_angle", param_.g_param.max_steering_angle, M_PI/20);
    nh.param<double>("global_planner/start_smoothing_distance", param_.g_param.start_smoothing_distance, 8);
    nh.param<double>("global_planner/corridor_width_min", param_.g_param.corridor_width_min, 1);
    nh.param<double>("global_planner/corridor_width_dynamic_min", param_.g_param.corridor_width_dynamic_min, 8);
    nh.param<double>("global_planner/safe_distance", param_.g_param.safe_distance, 10);
    nh.param<double>("global_planner/nominal_acceleration", param_.g_param.nominal_acceleration, 0.5);
    nh.param<double>("global_planner/object_velocity_threshold", param_.g_param.object_velocity_threshold, 0.1);
    nh.param<double>("global_planner/max_obstacle_prediction_query_size", param_.g_param.max_obstacle_prediction_query_size, 1000);


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
    nh.param<int>("local_planner/N_corr",param_.l_param.N_corr,51);
    nh.param<bool>("local_planner/isRearWheel",param_.l_param.isRearWheeled,true);
    nh.param<double>("local_planner/same_wpts_distance",param_.l_param.sameWptsDistance,0.01);
    nh.param<double>("local_planner/sfc_margin",param_.l_param.sfcMargin,0.0);
    nh.param<double>("local_planner/dyn_obst_range",param_.l_param.dynObstRange,30.0);

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
    nh.param<double>("global_planner/object_velocity_threshold",param_.p_param.staticCriteria,0.04); // just fix 1
    nh.param<string>("predictor/log_dir",param_.p_param.log_dir,"/home/jbs/test_ws/src/atypical_driving_snu/log/predictor");

    // Common
    nh.param<double>("goal_thres",param_.l_param.goalReachingThres,0.4); // just fix 1
    p_base->goal_thres = param_.l_param.goalReachingThres;

    nh.param<bool>("use_nominal_obstacle_rad",use_nominal_obstacle_radius,false); // just fix 1
    nh.param<bool>("use_keti_velocity",use_keti_velocity,true); // just fix 1
    param_.g_param.use_keti_velocity = use_keti_velocity; // when jungwon query isObject
    param_.p_param.predictWithKeti = use_keti_velocity; // when the target is predicted


    if(use_nominal_obstacle_radius)
        ROS_INFO("[SNU_PLANNER/RosWrapper] We assume fixed-size obstacle.");


    while (ros::Time::now().toSec()== 0 ){
        ROS_WARN("[SNU_PLANNER/RosWrapper] current ros time is zero. is running in sim time mode? Idling until valid ros clock");
        ros::Rate(20).sleep();
    }
    ROS_INFO("[SNU_PLANNER/RosWrapper] Initialized clock with ROS time %f",ros::Time::now().toSec());
    t0 = ros::Time::now().toSec();


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
        double dt = param.l_param.tStep;
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
    curGoal.point.x = p_base->goalXYSNU(0);
    curGoal.point.y = p_base->goalXYSNU(1);
    pubCurGoal.publish(curGoal);


//    auto pose = p_base->getCurPose();
//    pose.header.frame_id = SNUFrameId;


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


    // Nominal speed
    std_msgs::Float64 nominal_speed;
    nominal_speed.data = p_base->laneSpeed;
    pubNominalVelocity.publish(nominal_speed);

    // 2. topics which is not obtained from planning thread
    visualization_msgs::MarkerArray observations;
    geometry_msgs::PoseArray observationPose;
    observationPose.header.frame_id = SNUFrameId;

    geometry_msgs::PoseArray predictionPoseArray;
    predictionPoseArray.header.frame_id = SNUFrameId;
    // Predictor publish
    int nsId = 0;
    double curTime_ = curTime();
    VectorXf tEvalPred = VectorXf::LinSpaced(6,curTime_,param.l_param.horizon+curTime_);

//    cout <<"current predictor size in prepare = " << p_base->indexedPredictorSet.size() << endl;
    for(auto idPredictor : p_base->indexedPredictorSet){
        // observation
        observations.markers.push_back(get<1>(idPredictor).get_obsrv_marker(SNUFrameId,nsId++));
        for (auto pose : get<1>(idPredictor).get_obsrv_pose(SNUFrameId).poses){
            observationPose.poses.push_back(pose);
        }

        // prediction
        if (get<1>(idPredictor).is_prediction_available())
            for (auto pose : get<1>(idPredictor).eval_pose_seq(tEvalPred))
                predictionPoseArray.poses.push_back(pose);
    }

    pubDetectedObjectsPoseArray.publish(predictionPoseArray);
    pubObservationPoseArray.publish(observationPose);

    ROS_DEBUG("Number of predictor = %d",p_base->indexedPredictorSet.size());
    ROS_DEBUG("last id  = %d",nsId);

    pubObservationMarker.publish(observations);

    obstaclePrediction.markers.clear();
    nsId = 0;

    // Flush
    obstacleVelocityText.markers.clear();

    // obstacles registered to p_base and fed into local planner
    for(auto obstPath : p_base->getCurObstaclePathArray().obstPathArray){
        // per a obstacle path, make up marker array
        visualization_msgs::Marker m_obstacle_rad;

        double x0 = obstPath.obstPath[0].q(0);
        double y0 = obstPath.obstPath[0].q(1);

        // Current obstacle chain velocity information
        visualization_msgs::Marker InfoText;
        InfoText.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        InfoText.text = "[Fitting] vx=" + to_string(obstPath.constantVelocityXY(0)) +
                " / vy=" + to_string(obstPath.constantVelocityXY(1)) +
                " / v =" + to_string(obstPath.constantVelocityXY.norm()) +
                " \n [KETI mean] vx = "+ to_string(obstPath.meanVelocity(0)) +
                " / vy=" + to_string(obstPath.meanVelocity(1)) +
                " / v =" + to_string(obstPath.meanVelocity.norm()) ;

        InfoText.header.frame_id =SNUFrameId;
        InfoText.pose.orientation.w = 1.0;
        InfoText.pose.position.x = x0-0.5;
        InfoText.pose.position.y = y0-0.5;
        InfoText.pose.position.z = 1.0;
        InfoText.color.a = 1.0;
        InfoText.color.g = 1.0;
        InfoText.color.r = 1.0;
        InfoText.color.b = 1.0;
        InfoText.scale.z = 0.8;
        InfoText.ns = to_string(nsId);
        InfoText.id = nsId;
        obstacleVelocityText.markers.push_back(InfoText);

        m_obstacle_rad.header.frame_id = SNUFrameId;
        m_obstacle_rad.pose.orientation.w = 1.0;

        float obstacleAvgSpeed;
        if (use_keti_velocity)
            obstacleAvgSpeed = obstPath.meanVelocity.norm();
        else
            obstacleAvgSpeed = obstPath.constantVelocityXY.norm();

        if (obstacleAvgSpeed > param.p_param.staticCriteria )
            m_obstacle_rad.color.r = 6.0, m_obstacle_rad.color.a = 0.1; // dynamic
        else
            m_obstacle_rad.color.b = 6.0, m_obstacle_rad.color.a = 0.4; // static

        m_obstacle_rad.type = 3;
        m_obstacle_rad.ns = to_string(nsId);
        int id = 0 ;
        for (auto pnt : obstPath.obstPath){
            m_obstacle_rad.id = id++;
            // position
            m_obstacle_rad.pose.position.x = pnt.q(0);
            m_obstacle_rad.pose.position.y = pnt.q(1);
//            m_obstacle_rad.pose.position.z = p_base->predictorSet[0].getHeight();
            m_obstacle_rad.pose.position.z = 0.1;

            // rotation
            SE3 rot; rot.setIdentity();
            rot.rotate(AngleAxisd(pnt.theta,Vector3d::UnitZ()));
            Quaterniond q(rot.rotation());

            m_obstacle_rad.pose.orientation.x  = q.x();
            m_obstacle_rad.pose.orientation.y  = q.y();
            m_obstacle_rad.pose.orientation.z  = q.z();
            m_obstacle_rad.pose.orientation.w  = q.w();

            m_obstacle_rad.scale.x = 2*pnt.r1;
            m_obstacle_rad.scale.y = 2*pnt.r2;
            m_obstacle_rad.scale.z = 0.6;
            obstaclePrediction.markers.push_back(m_obstacle_rad);
        }
        nsId++;
    }

    {
        corridorSeq.markers.clear();
        for(int i = 0; i < p_base->corridor_seq.size(); i++){
            visualization_msgs::Marker marker;
            marker.header.frame_id = SNUFrameId;
            marker.id = i;
            marker.ns = std::to_string(i);
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = (p_base->corridor_seq[i].xl + p_base->corridor_seq[i].xu)/2;
            marker.pose.position.y = (p_base->corridor_seq[i].yl + p_base->corridor_seq[i].yu)/2;
            marker.pose.position.z = 0;
            marker.scale.x = p_base->corridor_seq[i].xu - p_base->corridor_seq[i].xl;
            marker.scale.y = p_base->corridor_seq[i].yu - p_base->corridor_seq[i].yl;
            marker.scale.z = 0.1;
            marker.color.a = 0.1;
            marker.color.r = 0;
            marker.color.g = 1;
            marker.color.b = 0;
            corridorSeq.markers.emplace_back(marker);
        }
        for(int i = p_base->corridor_seq.size(); i < count_corridors; i++){
            visualization_msgs::Marker marker;
            marker.id = i;
            marker.action = visualization_msgs::Marker::DELETE;
            corridorSeq.markers.emplace_back(marker);
        }
        if(p_base->corridor_seq.size() > count_corridors){
            count_corridors = p_base->corridor_seq.size();
        }
    }

    pubPredictionArray.publish(obstaclePrediction); // <- why this is in here? It should go to publish() (jungwon)
}

/**
 * @brief publish the car input and visualization markers
 * @details Do not use p_base here !!
 */
void RosWrapper::publish() {
    // 0. map publish

    /**
    if (isPCLReceived){
        sensor_msgs::PointCloud2 filtered_pcl;
        sensor_msgs::PointCloud2 ground_pcl;
        pcl::toROSMsg(*processedPclPtr,filtered_pcl);
        pcl::toROSMsg(*groundPclPtr,ground_pcl);

        filtered_pcl.header.stamp = ros::Time::now();
        filtered_pcl.header.frame_id = processedPclPtr->header.frame_id;

        ground_pcl.header.stamp = ros::Time::now();
        ground_pcl.header.frame_id = processedPclPtr->header.frame_id;
        pubFilteredPcl.publish(filtered_pcl);
        pubGroundPcl.publish(ground_pcl);
    }

**/

    // 1. Actuation command
    if (p_base->isLPsolved) {
        if (p_base->isLPPassed){
            auto cmd = p_base->getCurInput(curTime());
            cmd.header.stamp = ros::Time::now();
            cmd.steer_angle_cmd *= (180.0/3.14); // cmd output deg

            p_base->lastPublishedInput.twist.linear.x  = cmd.accel_decel_cmd;
            p_base->lastPublishedInput.twist.angular.z = cmd.steer_angle_cmd*3.14/180.0;
            p_base->lastPublishedInput.header.stamp = ros::Time::now();
            p_base->lastPublishedInput.header.frame_id = baseLinkId;

            pubCurCmd.publish(cmd);
            pubLastPublishedInput.publish(p_base->lastPublishedInput);

            geometry_msgs::Twist cmdDabin;
            pubMPCTraj.publish(MPCTraj);
            p_base->log_state_input(curTime());

            std_msgs::Float64 cmdSteer;
            cmdSteer.data = cmd.steer_angle_cmd;
            pubCurCmdSteer.publish(cmdSteer);

            std_msgs::Float64 cmdAcc;
            cmdAcc.data = cmd.accel_decel_cmd;
            pubCurCmdAcc.publish(cmdAcc);
        }
        else{
            ROS_WARN_THROTTLE(0.2,"MPC failed at current step. cmd will be zero.");

            driving_msgs::VehicleCmd cmd;
            cmd.accel_decel_cmd = 0 ;
            cmd.steer_angle_cmd = 0;
            cmd.header.stamp = ros::Time::now();
            pubCurCmd.publish(cmd);
            pubMPCTraj.publish(MPCTraj);
            p_base->log_state_input(curTime());
        }
        // added
        pubMPCTrajMarker.publish(p_base->mpc_result.getMPC(SNUFrameId));
    }

    pubOurOccu.publish(p_base->localMap);


    pubPath.publish(planningPath);
    pubCorridorSeq.publish(corridorSeq);
    pubTextTrackingVelocity.publish(obstacleVelocityText);
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

        ros::AsyncSpinner spinner(4);
        spinner.start();

        while(ros::ok()){

            if (p_base->isReached){
                ROS_INFO("[RosWrapper] got reach signal. Exising ROS wrapper.");
                return ;
            }
            updatePredictionModel();
            prepareROSmsgs(); // you may trigger this only under some conditions
            publish();
            processTf();
//            ros::spinOnce();
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


void RosWrapper::pclCallback(const sensor_msgs::PointCloud2::ConstPtr pcl_msg){

//    cout <<"In pcl callback : " << endl;

    auto t0 = ros::Time::now();
    isPCLReceived = true;
    processedPclPtr->header.frame_id = pcl_msg->header.frame_id; // keep the header as the velodyne
    processedPclPtr->points.clear();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCandidateGround (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudtotal (new pcl::PointCloud<pcl::PointXYZ>);
    vector<dbscan::Point> dbscanPoints;
    // We first crop xy
    for(int c = 0 ; c<pcl_msg->width ; c++)
        for(int r = 0 ; r<pcl_msg->height ; r++){
            // express the points in map frame
            geometry_msgs::Point p;
            pixelTo3DPoint(*pcl_msg,c,r,p);

            // like filter
            if (p.y < param.g_param.pcl_ly/2.0 and p.y > -param.g_param.pcl_ly/2.0 and
                p.x < param.g_param.pcl_lx/2.0 and p.x > -param.g_param.pcl_lx/2 ) {

                pcl::PointXYZ pclPnt;
                pclPnt.x = p.x;
                pclPnt.y = p.y;
                pclPnt.z = p.z;

                cloudtotal->points.push_back(pclPnt);
            }
        }


    auto t1 = ros::Time::now();

//    cout << "[PCL] cropping: " <<(t1 - t0).toSec() << endl;
    // 1. speckle removal

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
//    outrem.setInputCloud(cloudtotal);
//    outrem.setRadiusSearch(param.g_param.pcl_dbscan_eps);
//    outrem.setMinNeighborsInRadius (param.g_param.pcl_dbscan_minpnts);
//    outrem.filter(*cloudtotal);



    for (auto & pnt : cloudtotal->points){
        if (pnt.z < param.g_param.pcl_z_min)
            cloudCandidateGround->points.push_back(pnt);
        if (not param.g_param.use_ransac){
            if (pnt.z > param.g_param.pcl_z_min)
                processedPclPtr->points.push_back(pnt);
        }
    }




    if (not param.g_param.use_ransac){
        groundPclPtr->points = cloudCandidateGround->points;
    }else {

        // 2. extract the ground model by RANSAC
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(param.g_param.ransac_distance_threshold);
        seg.setMaxIterations(200);
        seg.setInputCloud(cloudCandidateGround);
        seg.segment(*inliers, *coefficients);

        groundPclPtr->points.clear();

//        ROS_INFO("[RosWrapper] inliners : [%d,%d]", inliers->indices.size(), cloudCandidateGround->points.size());

        double a = coefficients->values[0];
        double b = coefficients->values[1];
        double c = coefficients->values[2];
        double d = coefficients->values[3];
        Vector4f normVec(a, b, c, d);
        for (const auto &pnt : cloudtotal->points) {

            double dist = abs(coefficients->values[0] * pnt.x + coefficients->values[1] * pnt.y +
                              coefficients->values[2] * pnt.z + coefficients->values[3]) / normVec.norm();

            if (dist < param.g_param.ransac_ground_offest)
                groundPclPtr->points.push_back(pnt);
            else
                processedPclPtr->points.push_back(pnt);
        }


        auto t2 = ros::Time::now();

//        cout << "[PCL] ransac " <<(t2 - t1).toSec() << endl;


        // 3. another spekcle removal
        vector<int> indices;
        pcl::removeNaNFromPointCloud(*processedPclPtr,*processedPclPtr,indices);
        outrem.setInputCloud(processedPclPtr);
        outrem.setRadiusSearch(param.g_param.pcl_dbscan_eps);
        outrem.setMinNeighborsInRadius(param.g_param.pcl_dbscan_minpnts);
        outrem.filter(*processedPclPtr);
        auto t3 = ros::Time::now();

//        cout << "[PCL] speckle " << (t3 - t2).toSec() << endl;

    }


    processedPclPtr->header.frame_id = pcl_msg->header.frame_id; // keep the header as the velodyne
    groundPclPtr->header.frame_id = pcl_msg->header.frame_id; // keep the header as the velodyne


    sensor_msgs::PointCloud2 filtered_pcl;
    sensor_msgs::PointCloud2 ground_pcl;
    pcl::toROSMsg(*processedPclPtr,filtered_pcl);
    pcl::toROSMsg(*groundPclPtr,ground_pcl);

    filtered_pcl.header.stamp = ros::Time::now();
    filtered_pcl.header.frame_id = processedPclPtr->header.frame_id;

    ground_pcl.header.stamp = ros::Time::now();
    ground_pcl.header.frame_id = processedPclPtr->header.frame_id;
    pubFilteredPcl.publish(filtered_pcl);
    pubGroundPcl.publish(ground_pcl);
//    cout << "pcl processing time = " << (ros::Time::now() - t0).toSec() << endl;

}

void RosWrapper::cbDetectedObjects(const driving_msgs::DetectedObjectArray &objectsArray) {
    if (isCarPoseCovReceived)
        for(auto object : objectsArray.objects) {


            bool valueCheck = true;
            valueCheck = not(object.odom.pose.pose.position.x == 0 and object.odom.pose.pose.position.y == 0  );
            if (not valueCheck)
                continue;


            // Convert object pose into SNU frame

            string obstacleRefFrame = detectedObjectId;

            geometry_msgs::PoseStamped objectOrig; // pose w.r.t obstacleRefFrame
            objectOrig.header.frame_id = obstacleRefFrame;
            objectOrig.pose = object.odom.pose.pose;
            float vx = object.odom.twist.twist.linear.x;
            float vy = object.odom.twist.twist.linear.y;

            geometry_msgs::PoseStamped objectSNU; // pose w.r.t SNU frame

            try {
                tf_ls.transformPose(SNUFrameId,ros::Time(0),objectOrig,obstacleRefFrame,objectSNU);


                // Rotating the KETI velocity
                tf::StampedTransform T_sv; // T_{snuframe, velodyne frame}
                tf_ls.lookupTransform(SNUFrameId,obstacleRefFrame,ros::Time(0),T_sv);

                auto R_sv = T_sv.getBasis();
                tf::Vector3 velKetiOrig(vx,vy,0); // before rotation
                tf::Vector3 velKetiSNU  = R_sv*velKetiOrig;
                float vxSNU = velKetiSNU.x();
                float vySNU = velKetiSNU.y();

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
                    get<1>(*predictorOwner).update_observation(curTime(), objectSNU.pose,
                                                               updateDimension,vxSNU,vySNU);
                } else {
                    // no owner found, we create predictor and attach it
                    auto newPredictor = make_tuple(id, p_base->predictorBase);
                    get<1>(newPredictor).setIndex(id);
                    get<1>(newPredictor).update_observation(curTime(), objectSNU.pose,
                                                            updateDimension,vxSNU,vySNU);
                    p_base->indexedPredictorSet.push_back(newPredictor);
                    ROS_INFO("[SNU_PLANNER/RosWrapper] Predictor attached for obstacle id = %d", id);
                }

            }
            catch(tf::TransformException ex){
                ROS_ERROR("[SNU_PLANNER/RosWrapper] cannot register the object pose. "
                          "No tf connecting SNU frame with header of object. Callback ignored.");

                return;
            }

        }
    else
        ROS_WARN_THROTTLE(2,"[SNU_PLANNER/RosWrapper] Received object information. But the state of the car is not received");
}


/**
 * @brief receive the car pose Cov and update
 * @param dataPtr
 */
void RosWrapper::cbCarPoseCov(geometry_msgs::PoseWithCovarianceConstPtr dataPtr) {
//    cout << "in pose callback" << endl;
    // First, generate the ref frame based on the first-received pose
    if (not isFrameRefReceived){
        if (dataPtr->pose.position.x == 0 and dataPtr->pose.position.y == 0 ){
            ROS_ERROR("[SNU_PLANNER/RosWrapper] The first received pose is just zero. It is ignored as the ref tf");
            return;
        }

        p_base->Tws.setIdentity(); // just initialization
        Eigen::Quaterniond quat;
        Eigen::Vector3d transl;

        transl(0) = dataPtr->pose.position.x ;
        transl(1) = dataPtr->pose.position.y;
        transl(2) = dataPtr->pose.position.z;

        tw0 = transl;

        // due to jungwon
        quat.x() =  dataPtr->pose.orientation.x;
        quat.y() =  dataPtr->pose.orientation.y;
        quat.z() =  dataPtr->pose.orientation.z;
        quat.w() =  dataPtr->pose.orientation.w;

        p_base->Tws.translate(Vector3d::Zero());
        p_base->Tws.rotate(quat);

        ROS_INFO("[SNU_PLANNER/RosWrapper] Reference tf has been initialized with [%f,%f,%f,%f,%f,%f,%f]",
                tw0(0),tw0(1),tw0(2),quat.x(),quat.y(),quat.z(),quat.w());
        isFrameRefReceived = true;

        // If ref frame was set, then transform the laneNode
        if (isLaneRawReceived){
            Vector3d goalXYSNU(p_base->goal_x,p_base->goal_y,0); // goal w.r.t SNu frame
            goalXYSNU = p_base->Tws.rotation().transpose()*(goalXYSNU-tw0);
            p_base->goalXYSNU = goalXYSNU;

            auto lane_w = p_base->getLanePath(); // w.r.t world frame
            SE3 TwsWithTranslation;
            TwsWithTranslation.setIdentity();
            TwsWithTranslation.translate(tw0);
            TwsWithTranslation.rotate(p_base->Tws.rotation());
            lane_w.applyTransform(TwsWithTranslation.inverse());
            p_base->setLanePath(lane_w); // w.r.t SNU frame
            ROS_INFO("[SNU_PLANNER/RosWrapper] Lane-path transform completed!");
            p_base->laneOrig = Lane(lane_w); // should be applied transform
//            p_base->laneOrig.untilGoal(goalXYSNU(0),goalXYSNU(1)); // we keep only until the goal point

            if (p_base->laneOrig.points.size())
            isLaneReceived = true ;
            else{
                isLaneReceived = false;
                ROS_ERROR("Lane has no points. Maybe final goal position is not valid");
            }

        }else{
            ROS_ERROR("Tried transforming Lane raw in the pose callback. But no lane exsiting");
        }
        isGoalReceived = true;
    }
    // Retreive Tsb
    if (isCarSpeedReceived) {

        // 1. Converting car pose w.r.t Tw0
        auto poseOrig = dataPtr->pose;
        poseOrig.position.x -= tw0(0);
        poseOrig.position.y -= tw0(1);
        poseOrig.position.z -= tw0(2);


        auto Tw1 = DAP::pose_to_transform_matrix(poseOrig).matrix(); // Tw1
//        cout <<  p_base->Tws.inverse().matrix()<< endl;
        auto T01 = p_base->Tws.matrix().transpose() * Tw1; // no translation component in Tws
        auto poseTransformed = DAP::transform_matrix_to_pose(SE3(T01));


//        // Update the pose information w.r.t Tw1
//        geometry_msgs::PoseStamped poseStamped;
//        poseStamped.header.frame_id = SNUFrameId;
//        poseStamped.pose = poseTransformed;
//        poseStamped.header.stamp = ros::Time::now();
//
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

        // make
        curState.v = speed; // reverse gear = negative


        // tf broadcasting

        auto pose = p_base->getCurPose();

        tf::Transform transform;
        transform.setOrigin( tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) );

        q.setX(pose.pose.orientation.x);
        q.setY(pose.pose.orientation.y);
        q.setZ(pose.pose.orientation.z);
        q.setW(pose.pose.orientation.w);
        transform.setRotation(q);

        tf_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),SNUFrameId, baseLinkId));


        // publishing pose
        pose.header.frame_id = SNUFrameId;
        pose.header.stamp = ros::Time::now();
        pubCurPose.publish(pose);


        /**
         * MUTEX - upload
         */
//         p_base->mSet[0].lock();
         p_base->Tsb = T01;
         p_base->cur_pose.pose = poseTransformed;
         p_base->cur_pose.header.stamp = ros::Time::now();
         p_base->setCurPose(p_base->cur_pose);
         p_base->setCarState(curState);
//         p_base->mSet[0].unlock();


         isCarPoseCovReceived = true;
        ROS_INFO_ONCE("[SNU_PLANNER/RosWrapper] First received car state! ");
    }else {
        ROS_WARN("[SNU_PLANNER/RosWrapper] Car speed is not being received. CarState will not be updated");
    }
}

void RosWrapper::cbOccuMap(const nav_msgs::OccupancyGrid & occuMap) {

    p_base->localMapBuffer = occuMap;
//    ROS_INFO_STREAM_ONCE("[SNU_PLANNER/RosWrapper] Received first occupancy map");
//    ROS_WARN_STREAM("[SNU_PLANNER/RosWrapper] Received occupancy map [t diff since last update = " << (ros::Time::now().toSec()-sibalBeforeOccu)  );
    isLocalMapReceived = true;
    sibalBeforeOccu = ros::Time::now().toSec();
}

void RosWrapper::cbOccuUpdate(const map_msgs::OccupancyGridUpdateConstPtr &msg) {
//    cout << "update callback!!" <<endl;
    int index = 0;
    for(int y=msg->y; y< msg->y+msg->height; y++){
        for(int x=msg->x; x< msg->x+msg->width; x++){
            p_base->localMapBuffer.data[ y*p_base->localMapBuffer.info.width + x ] = msg->data[ index++ ];
        }
    }
}





void RosWrapper::cbCarSpeed(const std_msgs::Float64& speed_) {
    // Incoming data is km/h
//    cout << "in pose callback" << endl;
    speed = speed_.data*1000.0/3600;
    isCarSpeedReceived = true;
    p_base->cur_state.v = speed; // reverse gear = negative
}
/**
 * @brief update the observation for obstacles
 * @param obstPose
 */
void RosWrapper::cbObstacles(const geometry_msgs::PoseStamped& obstPose) {

    ROS_INFO_ONCE("[SNU_PLANNER/RosWrapper] got first obstacle observation");
    double t = curTime();
    // p_base->predictorSet[0].update_observation(t,obstPose.pose.position); // Deprecated

}

void RosWrapper::cbImu(const sensor_msgs::Imu &imu) {
    ROS_INFO_ONCE("imu received! ");

    tf::Quaternion q;
    q.setX(imu.orientation.x);
    q.setY(imu.orientation.y);
    q.setZ(imu.orientation.z);
    q.setW(imu.orientation.w);
    q.normalize();

    double roll,pitch,yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    pitchAngleFromImu = pitch;

//    Tci.setOrigin(tf::Vector3(0,0,0));
//    Tci.setRotation(q);
    if (not isImuReceived) { // first received
        qImu0 = q;
        ROS_INFO("[RosWrapper] rotation is initialized from imu: RPY [%f, %f, %f]",roll,pitch,yaw);
        isImuReceived = true;
    }
    qImu = q;
}

/**
 * @brief Whether all the necessary inputs are received
 * @return true if all the necessary inputs are received
 */
bool RosWrapper::isAllInputReceived() {

    bool isOKKK = true;

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

    if (not isLocalMapReceived) {
        ROS_ERROR_THROTTLE(2,"[SNU_PLANNER/RosWrapper] Still no occupancy grid received.");
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
    cout << "predict with keti ? " <<param.p_param.predictWithKeti << endl;
    Predictor::TargetManager predictor(param.p_param.predictWithKeti,param.p_param.queueSize,param.p_param.zHeight,param.p_param.polyOrder);
    p_base_shared->predictorBase = predictor;
    p_base_shared->predictorBase.setExpiration(param.p_param.trackingTime);
    p_base_shared->predictorBase.setLogFileDir(param.p_param.log_dir);
    p_base_shared->predictorSet.push_back(predictor);


}
/**
 * @brief Run the two thread (Planner & ROS)
 */
void Wrapper::run(){
    // Create thread
    threadPlanner = thread(&Wrapper::runPlanning,this)
            ; cout << "Thread for planners has been created" << endl;
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
bool Wrapper::processLane(double tTrigger) {

    if (ros_wrapper_ptr->isLocalMapReceived) {

        // Occupancy map frame = octomapGenFrameId
        auto curOccupancyGrid = p_base_shared->localMap;
        Vector2d windowOrig(curOccupancyGrid.info.origin.position.x, curOccupancyGrid.info.origin.position.y);
        double windowWidth = curOccupancyGrid.info.width * curOccupancyGrid.info.resolution;
        double windowHeight = curOccupancyGrid.info.height * curOccupancyGrid.info.resolution;

        Vector3d windowOrigSNU = p_base_shared -> Tso * Vector3d(windowOrig(0),windowOrig(1),0);

//        ROS_INFO("orig window = [%f,%f]",windowOrigSNU(0),windowOrigSNU(1));
        CarState curCarState = p_base_shared->getCarState(); // SNU frame
        int idxSliceStart,idxSliceEnd;
        vector<Vector2d, aligned_allocator<Vector2d>> pathSliced = p_base_shared->laneOrig.slicing(curCarState, Vector2d(windowOrigSNU(0),windowOrigSNU(1)), windowWidth,
                                                                      windowHeight,idxSliceStart,idxSliceEnd);

        // Move to GP
        if (pathSliced.size() < 3){

            ROS_WARN("[Wrapper] No slicing left. processLane returns false");
            return false;

        }




        p_base_shared->mSet[1].lock();
        p_base_shared->laneSliced.points = pathSliced;
        p_base_shared->laneSliced.widths = vector<double>(p_base_shared->laneOrig.widths.begin()+idxSliceStart,p_base_shared->laneOrig.widths.begin()+idxSliceEnd+1);
//        p_base_shared->laneSpeed = vLaneRef;
//        p_base_shared->laneCurvature = meanCurv;
        p_base_shared->mSet[1].unlock();
//
        if (not ros_wrapper_ptr->isLaneSliceLoaded){
            ros_wrapper_ptr->isLaneSliceLoaded = true;
            ROS_INFO("[SNU_PLANNER] Starting lane slicing! ");
        }
        return true;

    }else {
        ROS_WARN_THROTTLE(2, "[SNU_PLANNER/Wrapper] Lane cannot be processed. No occupancy map received");
        return false;
    }

}

bool Wrapper::planGlobal(double tTrigger){
    // call global planner

    bool gpPassed = gp_ptr->plan(tTrigger);
    if (not p_base_shared->isGPsolved)
        p_base_shared->isGPsolved = gpPassed;

    // let's log //TODO: Now, we don't need to log corridor in global planner step
//    if (gpPassed) {
//        updateCorrToBase();
//    }

//    p_base_shared->log_corridor(tTrigger);

    return gpPassed;
}

bool Wrapper::planLocal(double tTrigger) {

//    // call global planner
    bool lpPassed = lp_ptr->plan(tTrigger); // TODO
//    if (not p_base_shared->isGPsolved)
    if((not p_base_shared->isLPsolved) and lpPassed) {
        p_base_shared->isLPsolved = true;
        ROS_INFO("[LocalPlanner] First sovled! ");

    }
    if ((not p_base_shared->isLPPassed) and lpPassed){

        ROS_INFO("[LocalPlanner] Recovered from failure!");

    }

    p_base_shared->isLPPassed = lpPassed;

//    // let's log
//    if (lpPassed) {
    if (p_base_shared->isLPsolved)

        updateMPCToBase();
//    }
//    p_base_shared->log_corridor(tTrigger,tTrigger + param.l_param.horizon);
    p_base_shared->log_mpc(tTrigger);
    return lpPassed;
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

    // initial stuffs
    double Tp = 4.0; // sec
    auto tCkpG = chrono::steady_clock::now(); // check point time
    auto tCkpL = chrono::steady_clock::now(); // check point time
    auto tCkpStatePrinter = chrono::steady_clock::now();
    bool doGPlan = true; // turn on if we have started the class
    bool doLPlan = true; // turn on if we have started the class
    bool didLplanByG = false; // Lp already done in GP loop
    bool isPlanPossible = false;
    bool isLaneSuccess = false;
    bool isGPSuccess = false;
    bool isLPSuccess = false;
    bool isAllGoalReach = false;


    tCkpStatePrinter = chrono::steady_clock::now(); // check point time
    while (ros::ok()){

        p_base_shared->localMap = p_base_shared->localMapBuffer;
        // 2. Check the condition for planning
            isPlanPossible = ros_wrapper_ptr->isAllInputReceived();
            didLplanByG = false;


        if (ros_wrapper_ptr->isCarPoseCovReceived and
            ros_wrapper_ptr->isFrameRefReceived and
            ros_wrapper_ptr->isCarSpeedReceived){

            ROS_INFO_ONCE("[Wrapper] All car states have been received!");
            if (chrono::steady_clock::now() - tCkpStatePrinter > std::chrono::duration<double>(param.g_param.period)) {
//                p_base_shared->getCarState().print();
                tCkpStatePrinter = chrono::steady_clock::now();

            }
            }

            if (isPlanPossible) {
                ROS_INFO_ONCE( "[Wrapper] planning thread started !!!!\n"); // TODO
//                ROS_WARN_STREAM("[SNU_PLANNER/RosWrapper] updated occupancy map");


                // Goal checking
                auto distToGoal = (Vector2d(p_base_shared->cur_state.x,p_base_shared->cur_state.y) -
                        Vector2d(p_base_shared->goalXYSNU(0),p_base_shared->goalXYSNU(1))).norm();
//                cout << "distance to goal" << distToGoal << endl;
                if (distToGoal < p_base_shared->goal_thres){
                    printf("=========================================================\n");
                    ROS_INFO("[SNU_PLANNER] Reached goal! exiting");
                    p_base_shared->isReached = true;
                    return ;
                }



                ROS_INFO_ONCE("[Wrapper] start planning!");
                // 3. Do planning (GP->LP) or LP only
                if (doGPlan) { // G plan

                    tCkpG = chrono::steady_clock::now(); // check point time
                    // Lane processing
                    isLaneSuccess = processLane(ros_wrapper_ptr->curTime());
                    if (isLaneSuccess){
//                        ROS_INFO("[Wrapper] lane extracted!");
                        ROS_INFO("[Wrapper] begin GP..");
                        isGPSuccess = planGlobal(ros_wrapper_ptr->curTime());

                        if (isGPSuccess) { // let's call LP
                            ROS_INFO_STREAM("[Wrapper] GP success! planning time for gp: " <<
                                                                                           std::chrono::duration_cast<std::chrono::microseconds>(
                                                                                                   chrono::steady_clock::now() -
                                                                                                   tCkpG).count() * 0.001
                                                    << "ms");
//                            ROS_INFO("[Wrapper] begin LP..");

                            auto tCkp_mpc = chrono::steady_clock::now();
                            isLPSuccess = planLocal(ros_wrapper_ptr->curTime()); // TODO time ?
                            if (not isLPSuccess)

//                                ROS_INFO_STREAM("[Wrapper] LP success! planning time for lp: " <<
//                                                                                               std::chrono::duration_cast<std::chrono::microseconds>(
//                                                                                                       chrono::steady_clock::now() -
//                                                                                                       tCkp_mpc).count() *
//                                                                                               0.001
//                                                                                               << "ms");
                                ROS_INFO_STREAM("[Wrapper] LP failed.");
//                                ROS_INFO_STREAM("[Wrapper] LP failed! planning time for lp: " <<
//                                                                                               std::chrono::duration_cast<std::chrono::microseconds>(
//                                                                                                       chrono::steady_clock::now() -
//                                                                                                       tCkp_mpc).count() *
//                                                                                               0.001
//                                                                                               << "ms");





                        } else // GP failed
                            ROS_INFO_STREAM("[Wrapper] GP failed.");

                    }else{ // Lane failed
                        ROS_WARN("[Wrapper] lane extraction failed..");
                    }
//                    isGPSuccess = false;
                } // G plan ended
                doGPlan = (chrono::steady_clock::now() - tCkpG > std::chrono::duration<double>(param.g_param.period));

            } else { // planning cannot be started
                ROS_WARN_THROTTLE(2,
                                  "[Wrapper] waiting planning input subscriptions.. (message print out every 2 sec)");
            }

//        ros::spinOnce();
        ros::Rate(100).sleep();
    }
}




