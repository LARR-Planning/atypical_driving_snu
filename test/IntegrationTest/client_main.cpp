//
// Created by jbs on 20. 5. 9..
//

//
// Created by jbs on 20. 5. 9..
//

#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include <ros/ros.h>

//virtual environment generator to test Global planner

static geometry_msgs::PoseWithCovariance curState;
static float speed;

void cbCarPoseCov(const nav_msgs::Odometry& odom){
    // update position
    curState.pose.position.x = odom.pose.pose.position.x;
    curState.pose.position.y = -odom.pose.pose.position.y; //TODO: fix /airsim_car_node/PhysXCar/odom_local_ned topic to match axis
    curState.pose.orientation.x = odom.pose.pose.orientation.x;
    curState.pose.orientation.y = odom.pose.pose.orientation.y;
    curState.pose.orientation.z = odom.pose.pose.orientation.z;
    curState.pose.orientation.w = odom.pose.pose.orientation.w;

    // update speed
    float vx = odom.twist.twist.linear.x;
    float vy = -odom.twist.twist.linear.y;

}

int main(int argc,char** argv) {
    ros::init(argc,argv,"virtual_environment_generator");
    ros::NodeHandle nh("~");

    ros::Subscriber subCarPoseCov = nh.subscribe("/airsim_car_node/PhysXCar/odom_local_ned",1,cbCarPoseCov);
    ros::Publisher pubCarPoseCov = nh.advertise<geometry_msgs::PoseWithCovariance>("/atypical_planning_test/car_pose_cov",1);
    ros::Publisher pubCarSpeed = nh.advertise<std_msgs::Float64>("/current_speed",1);


    ros::Rate rate(40);
    while(ros::ok()){
        pubCarPoseCov.publish(curState);
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}