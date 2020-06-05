#include <atypical_planner/Wrapper.h>
#include <third_party/Vectormap.h>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include <ros/ros.h>

#include "Eigen/Core"


using namespace Planner;
using namespace Eigen;

nav_msgs::OccupancyGrid curOccupancyGrid;
bool isMapReceived = false;

void cbOccupancy(const nav_msgs::OccupancyGridPtr msg){
    curOccupancyGrid = *msg;
    isMapReceived = true;
}



int main(int argc, char ** argv){


    /**
     * Define lane
     */

    Lane laneOrig;
    laneOrig.width = 10;
    // This code mock the vector map cases

    int N1 = 20, N2 = 10;

    VectorXf l1X(N1); l1X.setZero();
    VectorXf l1Y(N1); l1Y.setLinSpaced(N1,0,73);
    VectorXf l2X(N2); l2X.setLinSpaced(N2,0,49);
    VectorXf l2Y(N2); l2Y.setConstant(73);

    for (int n = 0 ; n < N1 ; n++){
        laneOrig.points.emplace_back(l1X(n),l1Y(n));
    }

    for (int n = 0 ; n < N2 ; n++){
        laneOrig.points.emplace_back(l2X(n),l2Y(n));
    }

    /**
     * ROS
     */

    ros::init(argc,argv,"lane_tester");
    ros::NodeHandle nh("~");
    ros::Subscriber subOccupancyGrid = nh.subscribe("occupancy_grid",2,cbOccupancy);
    ros::Publisher pubOrigLane = nh.advertise<nav_msgs::Path>("orignal_lane",1);
    ros::Publisher pubSlicedLane = nh.advertise<nav_msgs::Path>("sliced_lane",1);
    string map_frame = "map";

    nav_msgs::Path origLane = laneOrig.getPath(map_frame);

    ros::Rate rl(20);

    while (ros::ok()){

        if (isMapReceived){

            vector<Vector2d> path = getPath()

        }
        pubOrigLane.publish(origLane);

        ros::spinOnce();
        rl.sleep();
    }




}


