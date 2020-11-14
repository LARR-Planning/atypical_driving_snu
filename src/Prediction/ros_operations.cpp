#include "third_party/Prediction/ros_operations.hpp"
using namespace DAP;
using namespace Eigen;


nav_msgs::Path DAP::TXYZQuat_to_nav_msgs(const TXYZQuatTraj & traj,string frame_id){

    nav_msgs::Path path;
    path.header.frame_id = frame_id;
    geometry_msgs::PoseStamped pose_stamped;

    for(int i = 0 ; i < traj.cols() ; i++){
        VectorXf result = traj.block(0,i,8,1);
        pose_stamped.pose.position.x = result(1);
        pose_stamped.pose.position.y = result(2);
        pose_stamped.pose.position.z = result(3);
        float quat_norm = result.segment(4,4).norm();
        pose_stamped.pose.orientation.w = result(4)/quat_norm;
        pose_stamped.pose.orientation.x = result(5)/quat_norm;
        pose_stamped.pose.orientation.y = result(6)/quat_norm;
        pose_stamped.pose.orientation.z = result(7)/quat_norm;           
        path.poses.push_back(pose_stamped);  
    }    
    return path;
};

vector<geometry_msgs::Pose> DAP::TXYZQuat_to_pose_vector(const TXYZQuatTraj &traj) {
    vector<geometry_msgs::Pose> pose_vector;

    geometry_msgs::PoseStamped pose_stamped;

    for(int i = 0 ; i < traj.cols() ; i++){
        VectorXf result = traj.block(0,i,8,1);
        pose_stamped.pose.position.x = result(1);
        pose_stamped.pose.position.y = result(2);
        pose_stamped.pose.position.z = result(3);
        float quat_norm = result.segment(4,4).norm();
        pose_stamped.pose.orientation.w = result(4)/quat_norm;
        pose_stamped.pose.orientation.x = result(5)/quat_norm;
        pose_stamped.pose.orientation.y = result(6)/quat_norm;
        pose_stamped.pose.orientation.z = result(7)/quat_norm;
        pose_vector.push_back(pose_stamped.pose);
    }
    return pose_vector;

}

TransformMatrixd DAP::pose_to_transform_matrix(const geometry_msgs::Pose & pose){
    Eigen::Quaterniond quat;
    quat.x() = pose.orientation.x;
    quat.y() = pose.orientation.y;
    quat.z() = pose.orientation.z;
    quat.w() = pose.orientation.w;
     
    Eigen::Matrix3d rot_mat = quat.toRotationMatrix();
    Eigen::Vector3d transl_vec(pose.position.x,pose.position.y,pose.position.z);
    TransformMatrixd trans_mat;
    trans_mat.setIdentity();
    trans_mat.translate(transl_vec);
    trans_mat.rotate(rot_mat);
    return trans_mat;
}

geometry_msgs::Pose DAP::transform_matrix_to_pose(const TransformMatrixd & trans_mat){
    Eigen::Quaterniond quat(trans_mat.rotation());
    Eigen::Vector3d vec = trans_mat.translation();

    geometry_msgs::Pose pose;
    pose.position.x = vec(0);
    pose.position.y = vec(1);
    pose.position.z = vec(2);
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();
    return pose;
}


TXYZQuatTraj DAP::pose_traj_to_TXYZQuat_traj(const Eigen::VectorXf &time_seq, const vector<geometry_msgs::Pose> & pose_vec){
    assert(time_seq.size() == pose_vec.size() && "length of time and poses are should be same");
    TXYZQuatTraj t_xyz_q_traj(8,pose_vec.size());
    for(int i = 0 ; i < pose_vec.size() ; i++){
        t_xyz_q_traj(0,i) = time_seq[i];
        t_xyz_q_traj(1,i) = pose_vec[i].position.x;
        t_xyz_q_traj(2,i) = pose_vec[i].position.y;
        t_xyz_q_traj(3,i) = pose_vec[i].position.z;
        t_xyz_q_traj(4,i) = pose_vec[i].orientation.w;        
        t_xyz_q_traj(5,i) = pose_vec[i].orientation.x;
        t_xyz_q_traj(6,i) = pose_vec[i].orientation.y;
        t_xyz_q_traj(7,i) = pose_vec[i].orientation.z;                
    }
    return t_xyz_q_traj;
}

TXYZQuatTraj DAP::pose_traj_to_TXYZQuat_traj(const Eigen::VectorXf &time_seq, const geometry_msgs::PoseArray & pose_array){
    return pose_traj_to_TXYZQuat_traj(time_seq,pose_array.poses);
}

TXYZQuatTraj DAP::TXYZ_traj_to_TXYZQuat_traj(const TXYZTraj & traj){
    int N_pnt  = traj.cols();
    TXYZQuatTraj traj2(8,N_pnt); traj2.setZero(); traj2.block(4,0,1,N_pnt).array() = 1.0 ; // w = 1    
    for (int i = 0 ; i < N_pnt ; i++){ // point traverse 
        for (int j = 0 ; j < 4 ; j++) // txyz
            traj2(j,i) = traj(j,i);
    } 
    return traj2;
};

TXYZTraj DAP::TXYZQuat_traj_to_TXYZ_traj(const TXYZQuatTraj & traj){
    int N_pnt  = traj.cols();
    TXYZTraj traj2(4,N_pnt); traj2.setZero();     
    for (int i = 0 ; i < N_pnt ; i++){ // point traverse 
        for (int j = 0 ; j < 4 ; j++) // txyz
            traj2(j,i) = traj(j,i);
    } 
    return traj2;
};


visualization_msgs::Marker DAP::nav_msgs_to_marker(const nav_msgs::Path prior_path,float scale){

    visualization_msgs::Marker obsrv_markers;

    obsrv_markers.header.frame_id = prior_path.header.frame_id;
    obsrv_markers.type = visualization_msgs::Marker::SPHERE_LIST;
    obsrv_markers.pose.orientation.w = 1;
    obsrv_markers.scale.x = scale;
    obsrv_markers.scale.y = scale;
    obsrv_markers.scale.z = scale;
        
    obsrv_markers.color.a = 0.8;
    obsrv_markers.points.clear();
    for (int n =0;n<prior_path.poses.size();n++){
        obsrv_markers.points.push_back(prior_path.poses[n].pose.position);
    }
    return obsrv_markers;
};    // extract pnts from path 


visualization_msgs::Marker DAP::TXYZ_traj_to_pnt_marker(const TXYZTraj & traj,string frame_id,float scale){    // extract pnts from path 
    nav_msgs::Path path = TXYZQuat_to_nav_msgs(TXYZ_traj_to_TXYZQuat_traj(traj),frame_id);
    return nav_msgs_to_marker(path,scale);
}

