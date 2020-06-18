//
// Created by jbs on 20. 5. 9..
//

//
// Created by jbs on 20. 5. 9..
//

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <tf/tf.h>
#include <driving_msgs/DetectedObjectArray.h>

//virtual environment generator to test Global planner

static geometry_msgs::PoseWithCovariance curState;
static geometry_msgs::PoseStamped curPose;
static float speed;
static float steering_angle = 0; // at the initial, it is zero
static double obst_rad_x;
static double obst_rad_y;
static driving_msgs::DetectedObjectArray objectsArray; // object should be emitted based on object_pose
static driving_msgs::DetectedObject object;



// ned to enu
void cbCarPoseCov(const nav_msgs::Odometry& pose_ned){

    // curPose.header.frame_id = pose_ned.header.frame_id;
    curPose.header.frame_id = "/map";
    // odom_ned_msg.header.frame_id = world_frame_id_;
    // odom_ned_msg.child_frame_id = "/airsim/odom_local_ned"; // todo make param
    Eigen::Quaternionf quat;
    Eigen::Vector3f transl;
    transl(0) = pose_ned.pose.pose.position.x;
    transl(1) = pose_ned.pose.pose.position.y;
    transl(2) = pose_ned.pose.pose.position.z;
    quat.x() =  pose_ned.pose.pose.orientation.x;
    quat.y() =  pose_ned.pose.pose.orientation.y;
    quat.z() =  pose_ned.pose.pose.orientation.z;
    quat.w() =  pose_ned.pose.pose.orientation.w;

    Eigen::Matrix3f rotm_en;
    Eigen::Affine3f transform;
    rotm_en << 1,0,0,
            0,-1,0,
            0,0,-1;

    quat.normalize();
    transform.setIdentity();
    transform.translate(transl);
    transform.rotate(quat);

    transform.prerotate(rotm_en);
    transform.rotate(rotm_en.transpose()); // to enu

    // std::cout << transform.matrix() << std::endl;

    quat = Eigen::Quaternionf(transform.rotation());
    curPose.pose.position.x = transform.translation()(0);
    curPose.pose.position.y = transform.translation()(1);
    curPose.pose.position.z = transform.translation()(2);

    curPose.pose.orientation.w = quat.w();
    curPose.pose.orientation.x = quat.x();
    curPose.pose.orientation.y = quat.y();
    curPose.pose.orientation.z = quat.z();


    tf::Quaternion q;
    q.setX(curPose.pose.orientation.x);
    q.setY(curPose.pose.orientation.y);ros::Time::now().toSec();
    q.setZ(curPose.pose.orientation.z);
    q.setW(curPose.pose.orientation.w);

    tf::Transform Twc; Twc.setRotation(q);
    tf::Matrix3x3 Rwc = Twc.getBasis();
    tf::Vector3 e1 = Rwc.getColumn(0);



    // update position
    auto odom = pose_ned;
    curState.pose = curPose.pose;

    // update speed
    float vx = odom.twist.twist.linear.x;
    float vy = -odom.twist.twist.linear.y;
    speed = sqrt(pow(vx,2)+pow(vy,2));

    double innerProd = e1.x()*vx + e1.y()*vy;
    if (innerProd < 0 )
        speed *=-1;

}
// This callback gets the current accel cmd
// with assumption that the steering angle input is the actual value of steering angle of current time
void cbAccelCmd(const geometry_msgs::Twist& twist){
    steering_angle = twist.angular.z;
}
// convert the object pose into DetectedObjectArray
void cbObject(const geometry_msgs::PoseStamped& object_pose){
   objectsArray.objects[0].header.frame_id = "/map";
   objectsArray.objects[0].odom.pose.pose = object_pose.pose;
}

int main(int argc,char** argv) {
    ros::init(argc,argv,"virtual_environment_generator");
    ros::NodeHandle nh("~");
    nh.param<double>("obstacle_radius_x",obst_rad_x,0.8);
    nh.param<double>("obstacle_radius_y",obst_rad_y,0.8);

    object.dimensions.x = 2*obst_rad_x;
    object.dimensions.y = 2*obst_rad_y; // play with this! JBS
    // z value will not be used
    objectsArray.objects.push_back(object);

    ros::Subscriber subCarPoseCov = nh.subscribe("/airsim_car_node/PhysXCar/odom_local_ned",1,cbCarPoseCov);
    ros::Subscriber subAccelCmd = nh.subscribe("/accel_cmd",1,cbAccelCmd);
    ros::Subscriber subObject = nh.subscribe("/atypical_planning_test/obstacle_pose",1,cbObject);

    ros::Publisher pubCarPoseCov = nh.advertise<geometry_msgs::PoseWithCovariance>("/current_pose",1);
    ros::Publisher pubCarSpeed = nh.advertise<std_msgs::Float64>("/current_speed",1);
    ros::Publisher pubCurSteering = nh.advertise<std_msgs::Float64>("/current_steer_angle",1);
    ros::Publisher pubCurPoseStamped = nh.advertise<geometry_msgs::PoseStamped>("current_car_posestamped",1);
    ros::Publisher pubDetectedObjects =  nh.advertise<driving_msgs::DetectedObjectArray>("/detected_objects",1);

    ros::Rate rate(40);
    while(ros::ok()){
        std_msgs::Float64 curAngle;
        curAngle.data = steering_angle;

        std_msgs::Float64 curSpeed;
        curSpeed.data = speed*3600.0/1000.0; // m/s to km/h

        pubCarPoseCov.publish(curState);
        pubCurSteering.publish(curAngle);
        pubCarSpeed.publish(curSpeed);
        pubCurPoseStamped.publish(curPose);
        pubDetectedObjects.publish(objectsArray);

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}