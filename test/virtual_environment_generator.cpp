#include <atypical_planner/Wrapper.h>

//virtual environment generator to test Global planner
int main(int argc,char** argv) {
    ros::init(argc,argv,"virtual_environment_generator");
    ros::NodeHandle nh("~");

    ros::Publisher pubCarPoseCov = nh.advertise<geometry_msgs::PoseWithCovariance>("/atypical_planning_test/car_pose_cov",1);
    ros::Publisher pubDesiredCarPose = nh.advertise<geometry_msgs::Pose>("/atypical_planning_test/desired_car_pose",1);

    geometry_msgs::PoseWithCovariance curState;
    curState.pose.position.x = -26;
    curState.pose.position.y = 4.5;

    geometry_msgs::Pose desiredState;
    desiredState.position.x = -68;
    desiredState.position.y = 20;

    ros::Rate rate(1);
    while(ros::ok()){
        pubCarPoseCov.publish(curState);
        pubDesiredCarPose.publish(desiredState);
        rate.sleep();
    }

    return 0;
}
