#include "third_party/Prediction/target_manager.hpp"
using namespace DAP;
using namespace Predictor;

TargetManager::TargetManager(int queue_size,float z_value,int poly_order):queue_size(queue_size),z_value(z_value),poly_order(poly_order) {

    assert(poly_order > 0 or queue_size >0 && "Invalid initialization of target manager" );
    //obsrv_traj_for_predict_total = TXYZTraj(4,100000);

};

void TargetManager::update_observation(float t,geometry_msgs::Point target_xy,Vector3f dimensions_){
    dimensions = dimensions_;
    observations.push_back(std::make_tuple(t,Vector2f(target_xy.x,target_xy.y)));
    lastObservationTime = t;
    if (observations.size() > queue_size)
        observations.pop_front();
    //obsrv_traj_for_predict_total(0,size_history) = t;
    //obsrv_traj_for_predict_total(1,size_history) = target_xy.x;
    //obsrv_traj_for_predict_total(2,size_history) = target_xy.y;
    //obsrv_traj_for_predict_total(3,size_history) = z_value;
    size_history ++ ;    
}

void TargetManager::update_predict(){
    if (observations.size() > queue_size-1){

        VectorXf t_vals(observations.size());
        VectorXf x_vals(observations.size());
        VectorXf y_vals(observations.size());
        int i =0 ;
        for (auto it = observations.begin() ; it != observations.end() ; it++, i++){
            t_vals(i) = std::get<0>(*it);
            x_vals(i) = std::get<1>(*it)(0);
            y_vals(i) = std::get<1>(*it)(1);        
        }

        ROS_DEBUG_STREAM("Prediction: ");
        ROS_DEBUG_STREAM("t : ");
        ROS_DEBUG_STREAM(t_vals.transpose());
        ROS_DEBUG_STREAM("x : ");
        ROS_DEBUG_STREAM(x_vals.transpose()) ;
        ROS_DEBUG_STREAM("y : ") ;
        ROS_DEBUG_STREAM(y_vals.transpose());

        fit_coeff_x = polyfit(t_vals,x_vals,poly_order);
        fit_coeff_y = polyfit(t_vals,y_vals,poly_order);   

        int N_pnt = observations.size();
        obsrv_traj_for_predict = DAP::TXYZTraj(4,N_pnt);
        obsrv_traj_for_predict.block(0,0,1,N_pnt) = t_vals.transpose();
        obsrv_traj_for_predict.block(1,0,1,N_pnt) = x_vals.transpose();
        obsrv_traj_for_predict.block(2,0,1,N_pnt) = y_vals.transpose();
        obsrv_traj_for_predict.block(3,0,1,N_pnt).array() = z_value;        
        is_predicted = true;     
    }
}
// x dir = velocity 
geometry_msgs::Pose TargetManager::eval_pose(float t){
    geometry_msgs::Pose pose;
    // translation 
    pose.position.x = polyeval(fit_coeff_x,t); 
    pose.position.y = polyeval(fit_coeff_y,t);
    pose.position.z = z_value;
    
    float vel_x = polyeval_derivative(fit_coeff_x,t);
    float vel_y = polyeval_derivative(fit_coeff_y,t);
    
    // rotation 
    float theta = atan2(vel_y, vel_x);
    DAP::TransformMatrix mat = DAP::TransformMatrix::Identity(); 
    mat.rotate(Eigen::AngleAxisf(theta,Eigen::Vector3f::UnitZ()));

    Eigen::Quaternionf quat(mat.rotation());
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();   
    return pose;
}

vector<geometry_msgs::Pose> TargetManager::eval_pose_seq(vector<float> ts){
    vector<geometry_msgs::Pose> pose_seq(ts.size());
    int ind = 0;
    for (auto it = ts.begin();it != ts.end(); it++,ind++)
        pose_seq[ind] = eval_pose(*it);
    return pose_seq;
}
vector<geometry_msgs::Pose> TargetManager::eval_pose_seq(Eigen::VectorXf ts){
    vector<float> ts_vec(ts.data(),ts.data() + ts.size());
    return eval_pose_seq(ts_vec);
}

visualization_msgs::Marker TargetManager::get_obsrv_marker(string world_frame_id){    
    visualization_msgs::Marker marker;
    marker.header.frame_id = world_frame_id;
    if(is_predicted){
        marker = TXYZ_traj_to_pnt_marker(obsrv_traj_for_predict,world_frame_id,1.0);
    }else{
        if (not isNoPredictionWarnEmitted) {
            cout << "[TargetManager] No prediction performed. empty marker will be returned" << endl;
            isNoPredictionWarnEmitted = true;
        }
    }
    return marker;
}; // observation used for the latest prediction update


TargetManager::~TargetManager(){
    // cout << "[TargetManager] Destroyed with log file" << endl;
    //std::ofstream file("observation.txt");
    //if(file.is_open())
    //    // file << obsrv_traj_for_predict_total.transpose().block(0,0,size_history,4);
    //file.close();
}