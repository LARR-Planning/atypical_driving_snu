#ifndef ROS_OPS
#define ROS_OPS

#include "basic_operations.hpp"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

namespace DAP{
    // conversions 
    nav_msgs::Path pose_vector_to_nav_msgs(const vector<geometry_msgs::Pose> pose_vec,string frame_id);
    nav_msgs::Path TXYZQuat_to_nav_msgs(const TXYZQuatTraj & traj,string frame_id = " ");
    geometry_msgs::PoseArray pose_vector_to_pose_array(const vector<geometry_msgs::Pose> pose_vec,string frame_id);
    TransformMatrix pose_to_transform_matrix(const geometry_msgs::Pose & pose);
    geometry_msgs::Pose transform_matrix_to_pose(const TransformMatrix & trans_mat);
    vector<geometry_msgs::Pose> TXYZQuat_to_pose_vector(const TXYZQuatTraj& traj);
    vector<TransformMatrix> pose_vector_to_transform_matrix_vec(const vector<geometry_msgs::Pose> & pose);
    geometry_msgs::PoseArray transform_matrix_vec_to_pose_array(const vector<TransformMatrix> & transl_mat,string frame_id);    
    TXYZQuatTraj pose_traj_to_TXYZQuat_traj(const Eigen::VectorXf &time_seq, const vector<geometry_msgs::Pose> & pose_vec);
    TXYZQuatTraj pose_traj_to_TXYZQuat_traj(const Eigen::VectorXf &time_seq, const geometry_msgs::PoseArray & pose_array);        
    TXYZTraj pose_traj_to_TXYZ_traj(const Eigen::VectorXf &time_seq, const geometry_msgs::PoseArray & pose_array);    
    Eigen::Vector3f point_to_vec3f(const geometry_msgs::Point & point);
    TXYZQuatTraj TXYZ_traj_to_TXYZQuat_traj(const TXYZTraj & traj);
    TXYZTraj TXYZQuat_traj_to_TXYZ_traj(const TXYZQuatTraj & traj);
    visualization_msgs::MarkerArray get_bearing_arrow_marker(const geometry_msgs::PoseArray & target_array,
                                                const geometry_msgs::PoseArray & chaser_array,
                                                const visualization_msgs::Marker & arrow_primitive);
    visualization_msgs::Marker nav_msgs_to_marker(const nav_msgs::Path path,float scale = 0.16);    // extract pnts from path 
    visualization_msgs::Marker TXYZ_traj_to_pnt_marker(const TXYZTraj & traj,string frame_id,float scale = 0.16);    // extract pnts from path 



}
#endif