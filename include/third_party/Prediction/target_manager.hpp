#include "ros_operations.hpp"
#include <list>
#include <tuple>
#include <fstream>
#include <ros/ros.h>

using namespace std;
using namespace Eigen;
typedef Matrix<float,6,1> Vector6f;
namespace Predictor{
    class TargetManager{
        private:
            bool isNoPredictionWarnEmitted = false;
            std::list<std::tuple<float,Vector6f>> observations; // (t,state) where state = [x,y,qx,qy,qz,qw]

            Eigen::VectorXf fit_coeff_x;
            Eigen::VectorXf fit_coeff_y;
            Eigen::VectorXf fit_coeff_qx;
            Eigen::VectorXf fit_coeff_qy;
            Eigen::VectorXf fit_coeff_qz;
            Eigen::VectorXf fit_coeff_qw;

        float z_value;
            int queue_size; 
            int poly_order;
            bool is_predicted = false;
            DAP::TXYZQuatTraj obsrv_traj_full; // full state having the entire pose information
            DAP::TXYZQuatTraj obsrv_traj_for_predict;
            DAP::TXYZTraj obsrv_traj_for_predict_total;
            int size_history = 0;
            double lastObservationTime;
            double trackingExpirationTime;
            Vector3f dimensions;

        public:
            TargetManager () {};
            ~TargetManager();
            TargetManager(int queue_size,float z_value,int poly_order);
            void update_observation(float t , geometry_msgs::Pose target_pose,Vector3f dimensions_ );
            void update_predict();
            float getHeight() {return z_value;};
            geometry_msgs::Pose eval_pose(float t);
            vector<geometry_msgs::Pose> eval_pose_seq(vector<float> ts);            
            vector<geometry_msgs::Pose> eval_pose_seq(VectorXf ts);
            geometry_msgs::PoseArray get_obsrv_pose(string world_frame_id);
            bool is_prediction_available() {return is_predicted;};       
            visualization_msgs::Marker get_obsrv_marker(string world_frame_id, int  ns = 0); // observation used for the latest prediction update
            double getLastObservationTime(){return lastObservationTime;};
            void setExpiration(const double & T) {trackingExpirationTime =  T;};
            double getExpiration() const {return trackingExpirationTime;};
            Vector3f getLastDimensions() const{return dimensions;}
    };

    typedef tuple<uint, TargetManager >  IndexedPredictor;
}