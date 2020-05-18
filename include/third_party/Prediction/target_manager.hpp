#include "ros_operations.hpp"
#include <list>
#include <tuple>
#include <fstream>
#include <ros/ros.h>

using namespace std;
using namespace Eigen;

namespace Predictor{
    class TargetManager{
        private:
            bool isNoPredictionWarnEmitted = false;
            std::list<std::tuple<float,Vector2f>> observations; // (t,Point) 
            Eigen::VectorXf fit_coeff_x;
            Eigen::VectorXf fit_coeff_y;            
            float z_value;
            int queue_size; 
            int poly_order;
            bool is_predicted = false; 
            DAP::TXYZTraj obsrv_traj_for_predict;
            DAP::TXYZTraj obsrv_traj_for_predict_total;
            int size_history = 0;
            double lastObservationTime;
            double trackingExpirationTime;
            Vector3f dimensions;

        public:
            TargetManager () {};
            ~TargetManager();
            TargetManager(int queue_size,float z_value,int poly_order);
            void update_observation(float t , geometry_msgs::Point target_xy,Vector3f dimensions_ ); // TODO z
            void update_predict();
            float getHeight() {return z_value;};
            geometry_msgs::Pose eval_pose(float t);
            vector<geometry_msgs::Pose> eval_pose_seq(vector<float> ts);            
            vector<geometry_msgs::Pose> eval_pose_seq(VectorXf ts);   
            bool is_prediction_available() {return is_predicted;};       
            visualization_msgs::Marker get_obsrv_marker(string world_frame_id); // observation used for the latest prediction update
            double getLastObservationTime(){return lastObservationTime;};
            void setExpiration(const double & T) {trackingExpirationTime =  T;};
            double getExpiration() const {return trackingExpirationTime;};
            Vector3f getLastDimensions() const{return dimensions;}


    };

    typedef tuple<uint, TargetManager >  IndexedPredictor;
}