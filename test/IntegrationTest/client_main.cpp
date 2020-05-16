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

//virtual environment generator to test Global planner

static geometry_msgs::PoseWithCovariance curState;
static float speed;
static float steering_angle = 0; // at the initial, it is zero

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
    speed = sqrt(pow(vx,2)+pow(vy,2));
}
// This callback gets the current accel cmd
// with assumption that the steering angle input is the actual value of steering angle of current time
void cbAccelCmd(const geometry_msgs::Twist& twist){
    steering_angle = twist.angular.z;
}

int main(int argc,char** argv) {
    ros::init(argc,argv,"virtual_environment_generator");
    ros::NodeHandle nh("~");

    ros::Subscriber subCarPoseCov = nh.subscribe("/airsim_car_node/PhysXCar/odom_local_ned",1,cbCarPoseCov);
    ros::Subscriber subAccelCmd = nh.subscribe("/accel_cmd",1,cbAccelCmd);

    ros::Publisher pubCarPoseCov = nh.advertise<geometry_msgs::PoseWithCovariance>("/current_pose",1);
    ros::Publisher pubCarSpeed = nh.advertise<std_msgs::Float64>("/current_speed",1);
    ros::Publisher pubCurSteering = nh.advertise<std_msgs::Float64>("/current_steer_angle",1);

    ros::Rate rate(40);
    while(ros::ok()){
        std_msgs::Float64 curAngle;
        curAngle.data = steering_angle;

        std_msgs::Float64 curSpeed;
        curSpeed.data = speed;

        pubCarPoseCov.publish(curState);
        pubCurSteering.publish(curAngle);
        pubCarSpeed.publish(curSpeed);

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}