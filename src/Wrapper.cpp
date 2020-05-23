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
RosWrapper::RosWrapper(shared_ptr<PlannerBase> p_base_,mutex* mSet_):p_base(p_base_),nh("~"),mSet(mSet_){

    // Load lanemap from parser
    string csv_file; double laneWidth;
    nh.param<string>("lane_csv_file",csv_file,"catkin_ws/src/atypical_driving_snu/keti_pangyo_path3.csv");
    nh.param<double>("lane_width",laneWidth,2.5);
    p_base->parse_tool.get_Coorddata(csv_file); // This parse only the center information
//     p_base->parse_tool.display_result();
    // TODO is valid csv?
    p_base->setLanePath(p_base->parse_tool.get_lanepath());
    p_base->setLaneWidth(laneWidth);
    ROS_INFO("Setting %f as lane width ",laneWidth);

    /**
    // Or just directly define here for easy test
    LanePath lanePath;
     LaneNode l1, l2;
     int N1 = 20, N2 = 10;

     l1.width = 10;
     l2.width = 13;

     VectorXf l1X(N1);
     l1X.setZero();
     VectorXf l1Y(N1);
     l1Y.setLinSpaced(N1, 0, 73);
     VectorXf l2X(N2);
     l2X.setLinSpaced(N2, 0, 49);
     VectorXf l2Y(N2);
     l2Y.setConstant(73);

     for (int n = 0; n < N1; n++) {
         geometry_msgs::Point pnt;
         pnt.x = l1X(n);
         pnt.y = l1Y(n);
         l1.laneCenters.push_back(pnt);
     }

     for (int n = 0; n < N2; n++) {
         geometry_msgs::Point pnt;
         pnt.x = l2X(n);
         pnt.y = l2Y(n);
         l2.laneCenters.push_back(pnt);
     }
     lanePath.lanes.emplace_back(l1);
     lanePath.lanes.emplace_back(l2);
     p_base->setLanePath(lanePath);
    **/

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

    pubLaneNode = nh.advertise<nav_msgs::Path>("lane_path",1);
    pubCurGoal = nh.advertise<geometry_msgs::PointStamped>("global_goal",1);
    pubOctomapSNU = nh.advertise<octomap_msgs::Octomap>("octomap_snu",1);

    // Subscriber
    subCarPoseCov = nh.subscribe("/current_pose",1,&RosWrapper::cbCarPoseCov,this);
    subDesiredCarPose = nh.subscribe("desired_car_pose",1,&RosWrapper::cbDesiredCarPose,this);
    subGlobalMap = nh.subscribe("global_map",1,&RosWrapper::cbGlobalMap,this);
    subLocalMap = nh.subscribe("local_map",1,&RosWrapper::cbLocalMap,this);
    subCarSpeed = nh.subscribe("/current_speed",1,&RosWrapper::cbCarSpeed,this);
    //subExampleObstaclePose = nh.subscribe("obstacle_pose",1,&RosWrapper::cbObstacles,this);
    subDetectedObjects= nh.subscribe("/detected_objects",1,&RosWrapper::cbDetectedObjects,this);
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
    nh.param<string>("octomap_gen_frame_id",octomapGenFrameId,"/SNU");

    // Own
    planningPath.header.frame_id = SNUFrameId;
    if (octomapGenFrameId == (SNUFrameId)){
        ROS_INFO("[SNU_PLANNER/RosWrapper] SNU frame is identical with octomap frame.");
        p_base->To0.setIdentity();
        isOctomapFrameResolved = true;
    }

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
    nh.param<bool>("global_planner/is_world_snu_frame",param_.g_param.is_world_box_snu_frame,false);

    // local planner
    nh.param<double>("local_planner/horizon",param_.l_param.horizon,5);
    nh.param<double>("local_planner/ts",param_.l_param.tStep,0.1);
    nh.param<double>("local_planner/obstacle_radius_nominal",param_.l_param.obstRadiusNominal,0.3);
    nh.param<double>("local_planner/car_longtitude",param_.l_param.carLongtitude,2.7);
    nh.param<double>("local_planner/max_steer",param_.l_param.maxSteer,M_PI/30);
    nh.param<double>("local_planner/max_accel",param_.l_param.maxAccel,3);
    nh.param<double>("local_planner/min_accel",param_.l_param.minAccel,-1);
    bool isUseSimTimeMode = false;


    // predictor

    nh.param<int>("predictor/observation_queue",param_.p_param.queueSize,6);
    nh.param<float>("predictor/ref_height",param_.p_param.zHeight,1.0);
    nh.param<int>("predictor/poly_order",param_.p_param.polyOrder,1); // just fix 1
    nh.param<double>("predictor/tracking_expiration",param_.p_param.trackingTime,2.0); // just fix 1

    // Common
    double goal_x,goal_y;
    nh.param<double>("initial_goal/x",goal_x,0.0); // just fix 1
    nh.param<double>("initial_goal/y",goal_y,0.0); // just fix 1
    nh.param<double>("goal_thres",param_.l_param.goalReachingThres,0.4); // just fix 1

    nh.param<bool>("use_nominal_obstacle_rad",use_nominal_obstacle_radius,false); // just fix 1
    if(use_nominal_obstacle_radius)
        ROS_INFO("[SNU_PLANNER/RosWrapper] We assume fixed-size obstacle.");




    while (ros::Time::now().toSec()== 0 ){
        ROS_WARN("[SNU_PLANNER/RosWrapper] current ros time is zero. is running in sim time mode? Idling until valid ros clock");
        ros::Rate(20).sleep();
    }
    ROS_INFO("[SNU_PLANNER/RosWrapper] Initialized clock with ROS time %f",ros::Time::now().toSec());
    t0 = ros::Time::now().toSec();

    CarState goalState;
    goalState.x = goal_x;
    goalState.y = goal_y;
    p_base->setDesiredState(goalState); // TODO, convert it into our frame
    ROS_INFO("[SNU_PLANNER/RosWrapper] received global goal [%f,%f] (in global frame)",goal_x,goal_y);


    param = param_;
}

/**
 * @breif Convert the row information of p_base into rosmsgs
 * @details Extract information from p_base
 */
void RosWrapper::prepareROSmsgs() {

    // 1. Topics directly obtained from p_base
    if(mSet[1].try_lock()){
        // planning path
        planningPath.poses.clear();
        geometry_msgs::PoseStamped poseStamped;
        for (auto pose : p_base->getMPCResultTraj().xs){
            poseStamped.pose.position.x = pose.x;
            poseStamped.pose.position.y = pose.y;
            planningPath.poses.push_back(poseStamped);
        }

        // corridor_seq - jungwon
        corridorSeq.markers.clear();
        int marker_id = 0;
        double car_z_min = param.g_param.car_z_min;
        double car_z_max = param.g_param.car_z_max;

        visualization_msgs::Marker marker;
        marker.header.frame_id = SNUFrameId;
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
        // skeleton path - jungwon
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
        // search range - jungwon
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

        // MPC path
        if (p_base->isLPsolved) {
            MPCTraj.header.frame_id = SNUFrameId;
            MPCTraj.poses.clear();
            PoseStamped MPCPose;
            MPCResultTraj mpcResultTraj = p_base->getMPCResultTraj();
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



        mSet[1].unlock();
    }else{
//        ROS_WARN("[RosWrapper] Locking failed for ros data update. The output of p_base is being modified in planner ");
    }

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
    // e.g pub1.publish(topic1)
    if (p_base->isLPsolved) {
        pubCurCmd.publish(p_base->getCurInput(curTime()));
        pubMPCTraj.publish(MPCTraj);
    }
    pubPath.publish(planningPath);
    pubCorridorSeq.publish(corridorSeq);


    // Tranform broadcasting the tf of the current car w.r.t the first received tf

    if (isFrameRefReceived) {
        // send T01
        auto pose = p_base->getCurPose();

        tf::Transform transform;
        transform.setOrigin( tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) );
        tf::Quaternion q;
        q.setX(pose.pose.orientation.x);
        q.setY(pose.pose.orientation.y);
        q.setZ(pose.pose.orientation.z);
        q.setW(pose.pose.orientation.w);
        transform.setRotation(q);

        tf_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),SNUFrameId, "car_base_link"));

        // send Tw0
        transform.setOrigin( tf::Vector3(p_base->Tw0.translation()(0),
                p_base->Tw0.translation()(1),
                p_base->Tw0.translation()(2)));
        auto qd = Eigen::Quaterniond(p_base->Tw0.rotation());
        q.setX(qd.x());
        q.setY(qd.y());
        q.setZ(qd.z());
        q.setW(qd.w());
        transform.setRotation(q);//
        tf_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),worldFrameId,SNUFrameId));

        // Send octomap snu
        p_base->octomap_snu_msgs.header.frame_id = octomapGenFrameId;
        p_base->octomap_snu_msgs.header.stamp = ros::Time::now();
        pubOctomapSNU.publish(p_base->octomap_snu_msgs);
    }

    if (isLaneReceived){
        pubLaneNode.publish(p_base->getLanePath().getPath(SNUFrameId));
    }
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
            ros::spinOnce(); // callback functions are executed
            lr.sleep();
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

            string obstacleRefFrame = object.header.frame_id;
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
        p_base->Tw0.setIdentity();
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


        p_base->Tw0.translate(transl);
        p_base->Tw0.rotate(quat);
        p_base->T0s.setIdentity();
        p_base->T0s.rotate(quat);

        ROS_INFO("[SNU_PLANNER/RosWrapper] Reference tf has been initialized with [%f,%f,%f,%f,%f,%f,%f]",
                transl(0),transl(1),transl(2),quat.x(),quat.y(),quat.z(),quat.w());
//        ROS_INFO("[SNU_PLANNER/RosWrapper] Reference tf has been initialized with [%f,%f,%f,%f,%f,%f,%f]",
//                 transl(0),transl(1),transl(2),0.0,0.0,0.0,1.0);
        isFrameRefReceived = true;
        if(not isOctomapFrameResolved){
            ROS_INFO("[SNU_PLANNER/RosWrapper] seems that frame of octomap = map frame.");
            p_base->To0 = p_base->Tw0;
            isOctomapFrameResolved = true;
        }

        // If ref frame was set, then transform the laneNode
        if (isLaneRawReceived){
            auto lane_w = p_base->getLanePath(); // w.r.t world frame
            lane_w.applyTransform(p_base->Tw0.inverse());
            p_base->setLanePath(lane_w); // w.r.t SNU frame
            ROS_INFO("[SNU_PLANNER/RosWrapper] Lane-path transform completed!");
            isLaneReceived = true;
        }else{
            ROS_ERROR("Tried transforming Lane raw in the pose callback. But no lane exsiting");
        }

        // Convert the global goal to local goal
        CarState global_goal = p_base->getDesiredState();
        Vector4d xb(global_goal.x,global_goal.y,0,1);
        Vector4d xa = p_base->Tw0.inverse()*xb;
        global_goal.x = xa(0);
        global_goal.y = xa(1);
        p_base->setDesiredState(global_goal);

        ROS_INFO("[SNU_PLANNER/RosWrapper] received global goal [%f,%f] (in SNU frame)",global_goal.x,global_goal.y);
        isGoalReceived = true;
    }

    if (isCarSpeedReceived) {
        if (mSet[0].try_lock()) {
            ROS_INFO_ONCE("[SNU_PLANNER/RosWrapper] First received car state");
//            cout << "call back back" << endl;
            // Converting car pose w.r.t Tw0
            auto poseOrig = dataPtr->pose;
            SE3 Tw1 = DAP::pose_to_transform_matrix(poseOrig).cast<double>(); // Tw1
            SE3 T01 = p_base->Tw0.inverse() * Tw1;
            auto poseTransformed = DAP::transform_matrix_to_pose(T01.cast<float>());
            // Update the pose information w.r.t Tw1
            geometry_msgs::PoseStamped poseStamped;
            poseStamped.header.frame_id = SNUFrameId;
            poseStamped.pose = poseTransformed;
            p_base->setCurPose(poseStamped);
            p_base->setCurTf(T01);
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
            Twc.setRotation(q);

            tf::Matrix3x3 Rwc = Twc.getBasis();
            tf::Vector3 e1 = Rwc.getColumn(0);
            double theta = atan2(e1.y(), e1.x());
            curState.theta = theta;

            // make v
            curState.v = speed; // reverse gear = negative

            ROS_DEBUG("Current car state (x,y,theta(degree),v) : [%f,%f,%f,%f]", curState.x, curState.y,
                      curState.theta * 180 / M_PI, curState.v);

            p_base->setCarState(curState);
            mSet[0].unlock();
            isCarPoseCovReceived = true;
            //        ROS_INFO("[RosWrapper] car pose update");
//        }else{
////        ROS_WARN("[RosWrapper] callback for CarPoseCov locked by planner. Passing update");
//        }
        }
    }else {
        ROS_WARN("[SNU_PLANNER/RosWrapper] Car speed is not being received. CarState will not be updated");
    }
}

void RosWrapper::cbCarSpeed(const std_msgs::Float64 speed_) {
    speed = speed_.data;
    isCarSpeedReceived = true;

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
    if(mSet[0].try_lock()) {

        p_base->setLocalMap(dynamic_cast<octomap::OcTree *>(octomap_msgs::binaryMsgToMap(octomap_msg)));
        double xmin, ymin, zmin;
        double xmax, ymax, zmax;

        p_base->getLocalOctoPtr()->getMetricMin(xmin, ymin, zmin);
        p_base->getLocalOctoPtr()->getMetricMax(xmax, ymax, zmax);
        p_base->curOctomapMin = octomap::point3d(xmin, ymin, zmin);
        p_base->curOctomapMax = octomap::point3d(xmax, ymax, zmax);

        if (p_base->getLocalOctoPtr()->getNumLeafNodes() == 0) {
            ROS_ERROR("invalid octomap");
            return;
        }


//        octomap::point3d minPnt(-100,-100,-100);
//        octomap::point3d maxPnt(100,100,100);
//        p_base->getLocalOctoPtr()->setBBXMax(maxPnt);
//        p_base->getLocalOctoPtr()->setBBXMin(minPnt);

//         ROS_INFO("Octomap bbx = [%f, %f, %f] / [%f, %f, %f ]",minPnt.x(),minPnt.y(),minPnt.z(),
//                maxPnt.x(),maxPnt.y(),maxPnt.z());

        ROS_INFO_ONCE("Octomap loaded");

        mSet[0].unlock();
        isLocalMapReceived = true;
//        ROS_INFO("[RosWrapper] local map update");
//    }else{
//        ROS_WARN("[RosWrapper] callback for CarPoseCov locked by planner. Passing update");
//    }
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
    // p_base->predictorSet[0].update_observation(t,obstPose.pose.position); // Deprecated

}

/**
 * @brief Whether all the necessary inputs are received
 * @return true if all the necessary inputs are received
 */
bool RosWrapper::isAllInputReceived() {

    bool isOKKK = true;
    if (not isLocalMapReceived) {
        ROS_ERROR_THROTTLE(2,"[SNU_PLANNER/RosWrapper] Still no octomap received.");
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
    if (not isOctomapFrameResolved) {
        ROS_ERROR_THROTTLE(2,"[SNU_PLANNER/RosWrapper] Still we could not find the frame_id of octomap");
        return false;
    }

    if (not isLaneReceived) {
        ROS_ERROR_THROTTLE(2,"[SNU_PLANNER/RosWrapper] No lane information loaded");
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
    ros_wrapper_ptr = new RosWrapper(p_base_shared,mSet);
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




