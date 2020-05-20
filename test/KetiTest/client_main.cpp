//
// Created by jbs on 20. 5. 19..
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

// This code interfaces with KETI raw data
// see the data worlds/keti_dataset1 or https://drive.google.com/open?id=1ZB5CpZzc3AS7zI0zRIVNStMBK6quSDXF



bool isPoseReceieved = false;
static geometry_msgs::PoseWithCovariance curState;
static geometry_msgs::PoseStamped curPose;


// ned to enu
void cbCarPoseCov(const geometry_msgs::PoseStamped& pose){
    isPoseReceieved = true;
    curState.pose = pose.pose;
}

int main(int argc,char** argv) {
    ros::init(argc,argv,"virtual_environment_generator");
    ros::NodeHandle nh("~");

    ros::Subscriber subCarPoseCov = nh.subscribe("/current_pose_keti",1,cbCarPoseCov);
    ros::Publisher pubCarPoseCov = nh.advertise<geometry_msgs::PoseWithCovariance>("/current_pose",1);

    ros::Rate rate(40);
    while(ros::ok()){
        if (isPoseReceieved)
            pubCarPoseCov.publish(curState);
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
