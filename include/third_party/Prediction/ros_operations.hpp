#ifndef ROS_OPS
#define ROS_OPS

#include "basic_operations.hpp"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

namespace DAP{
    // conversions 
    nav_msgs::Path TXYZQuat_to_nav_msgs(const TXYZQuatTraj & traj,string frame_id = " ");
    TransformMatrixd pose_to_transform_matrix(const geometry_msgs::Pose & pose);
    geometry_msgs::Pose transform_matrix_to_pose(const TransformMatrixd & trans_mat);
    vector<geometry_msgs::Pose> TXYZQuat_to_pose_vector(const TXYZQuatTraj& traj);
    TXYZQuatTraj pose_traj_to_TXYZQuat_traj(const Eigen::VectorXf &time_seq, const vector<geometry_msgs::Pose> & pose_vec);
    TXYZQuatTraj pose_traj_to_TXYZQuat_traj(const Eigen::VectorXf &time_seq, const geometry_msgs::PoseArray & pose_array);        
    TXYZQuatTraj TXYZ_traj_to_TXYZQuat_traj(const TXYZTraj & traj);
    TXYZTraj TXYZQuat_traj_to_TXYZ_traj(const TXYZQuatTraj & traj);
    visualization_msgs::MarkerArray get_bearing_arrow_marker(const geometry_msgs::PoseArray & target_array,
                                                const geometry_msgs::PoseArray & chaser_array,
                                                const visualization_msgs::Marker & arrow_primitive);
    visualization_msgs::Marker nav_msgs_to_marker(const nav_msgs::Path path,float scale = 0.16);    // extract pnts from path 
    visualization_msgs::Marker TXYZ_traj_to_pnt_marker(const TXYZTraj & traj,string frame_id,float scale = 0.16);    // extract pnts from path 



}
#endif