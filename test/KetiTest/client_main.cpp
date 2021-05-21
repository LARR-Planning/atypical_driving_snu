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
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
// This code interfaces with KETI raw data
// see the data worlds/keti_dataset1 or https://drive.google.com/open?id=1ZB5CpZzc3AS7zI0zRIVNStMBK6quSDXF



bool isPoseReceieved = false;
bool isClockReceived = false;

static geometry_msgs::PoseWithCovariance curState;
static geometry_msgs::PoseStamped curPose;
static rosgraph_msgs::Clock  keti_clock;
static sensor_msgs::PointCloud2 pclIn;

using namespace pcl;

PointCloud<PointXYZ>::Ptr  cloud_ptr;
PointCloud<PointXYZ>::Ptr  cloud_filtered;

double rejectionRadius = 0;

// ned to enu
void cbCarPoseCov(const geometry_msgs::PoseStamped& pose){
    isPoseReceieved = true;
    curState.pose = pose.pose;
}
// For the publication of time
void cbPcl(const sensor_msgs::PointCloud2& velodyne){
    isClockReceived = true;

    cloud_filtered->clear();

    pcl::fromROSMsg(velodyne,*cloud_ptr);
    for (auto point : cloud_ptr->points){
        double dist = Eigen::Vector3d(point.x,point.y,point.z).norm();
        if (dist > rejectionRadius)
            cloud_filtered->points.push_back(point);
    }

    toROSMsg(*cloud_filtered,pclIn);

//    pclIn = velodyne;
    pclIn.header.stamp = ros::Time::now();
    pclIn.header.frame_id = velodyne.header.frame_id;

}

int main(int argc,char** argv) {
    ros::init(argc,argv,"virtual_environment_generator");
    ros::NodeHandle nh("~");
    nh.param("rejection_radius",rejectionRadius,0.1);

//    ros::Subscriber subCarPoseCov = nh.subscribe("/current_pose_keti",1,cbCarPoseCov);
    ros::Subscriber subPCL = nh.subscribe("/velodyne_points",1,cbPcl);
//    ros::Publisher pubCarPoseCov = nh.advertise<geometry_msgs::PoseWithCovariance>("/current_pose",1);
    ros::Publisher pubKetiClock = nh.advertise<rosgraph_msgs::Clock>("/clock",1);
    ros::Publisher pubPcl= nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_snu",1);

    cloud_filtered = static_cast<boost::shared_ptr<PointCloud<PointXYZ>>>(new PointCloud<PointXYZ>);
    cloud_ptr = static_cast<boost::shared_ptr<PointCloud<PointXYZ>>>(new PointCloud<PointXYZ>);
    ros::Rate rate(40);
    while(ros::ok()){
        if (isPoseReceieved) {
//            pubCarPoseCov.publish(curState);
        }
        if (isClockReceived)
            pubPcl.publish(pclIn);
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
