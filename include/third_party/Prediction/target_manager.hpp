#include "ros_operations.hpp"
#include <list>
#include <tuple>
#include <fstream>
#include <ros/ros.h>

using namespace std;
using namespace Eigen;
typedef Matrix<float,6,1> Vector6f;
typedef Matrix<float,8,1> Vector8f;

namespace Predictor{
    class TargetManager{
        private:
            int managerIdx;
            bool isNoPredictionWarnEmitted = false;
            std::list<std::tuple<float,Vector8f>> observations; // (t,state) where state = [x,y,qx,qy,qz,qw]

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
            DAP::TXYZTraj obst_traj_velocity_keti; // velocity history from keti
            DAP::TXYZTraj obsrv_traj_for_predict_total;

            int size_history = 2000;
            double lastObservationTime;
            double trackingExpirationTime;
            Vector3f dimensions;

            list<Vector8f> observationHistory;
            string logFileDir;
        public:
            TargetManager () {};
            ~TargetManager();
            TargetManager(int queue_size,float z_value,int poly_order,int index = 0 );
            void update_observation(float t , geometry_msgs::Pose target_pose,Vector3f dimensions_ ,float vxKeti  , float vyKeti);
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
            void setLogFileDir(string dir) {logFileDir = dir;};
            void setIndex(int idx) {managerIdx = idx;};
            Vector2f getFitVelocity () {return Vector2f(fit_coeff_x(1),fit_coeff_y(1)); };
            Vector2f getAvgKetiVelocity() {return Vector2f(obst_traj_velocity_keti.row(1).mean(),obst_traj_velocity_keti.row(2).mean());}

    };

    typedef tuple<uint, TargetManager >  IndexedPredictor;
}