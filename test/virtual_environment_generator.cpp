#include <atypical_planner/Wrapper.h>
#include "nav_msgs/Odometry.h"

//virtual environment generator to test Global planner

static geometry_msgs::PoseWithCovariance curState;

void cbCarPoseCov(const nav_msgs::Odometry& odom){
    curState.pose.position.x = odom.pose.pose.position.x;
    curState.pose.position.y = odom.pose.pose.position.y;
    curState.pose.orientation.x = odom.pose.pose.orientation.x;
    curState.pose.orientation.y = odom.pose.pose.orientation.y;
    curState.pose.orientation.z = odom.pose.pose.orientation.z;
    curState.pose.orientation.w = odom.pose.pose.orientation.w;
}

int main(int argc,char** argv) {
    ros::init(argc,argv,"virtual_environment_generator");
    ros::NodeHandle nh("~");

    ros::Subscriber subCarPoseCov = nh.subscribe("/airsim_car_node/PhysXCar/odom_local_ned",1,cbCarPoseCov);
    ros::Publisher pubCarPoseCov = nh.advertise<geometry_msgs::PoseWithCovariance>("/atypical_planning_test/car_pose_cov",1);
    ros::Publisher pubDesiredCarPose = nh.advertise<geometry_msgs::Pose>("/atypical_planning_test/desired_car_pose",1);

    double desired_x, desired_y;
    nh.param<double>("virtual_env/desired_x", desired_x, 20);
    nh.param<double>("virtual_env/desired_y", desired_y, 68);

    geometry_msgs::Pose desiredState;
    desiredState.position.x = desired_x;
    desiredState.position.y = desired_y;

    ros::Rate rate(40);
    while(ros::ok()){
        pubCarPoseCov.publish(curState);
        pubDesiredCarPose.publish(desiredState);
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
