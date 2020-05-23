//
// Created by jbs on 20. 4. 6..
//

#ifndef ATYPICAL_DRIVING_PLANNERBASE_H
#define ATYPICAL_DRIVING_PLANNERBASE_H

#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <vector>
#include <memory>
#include <geometry_msgs/Twist.h>
#include <mutex>
#include <thread>
#include <string>
#include <math.h>
#include <list>
#include <tuple>
#include <Eigen/Core>
#include <third_party/Prediction/target_manager.hpp>

#include <third_party/Vectormap.h>

#include <third_party/Utils.h>
#include <driving_msgs/VehicleCmd.h>

#include <third_party/parser.h>

using namespace std;
using namespace Eigen;


namespace Planner {
    /**
     * @brief
     */
    struct Corridor {
        double xl;
        double yl;
        double xu;
        double yu;
        double t_start;
        double t_end;
    };

    /**
     * @brief Parameters for local planner
     */
    struct ParamLocal {
        double horizon;
        double tStep;
        double obstRadiusNominal;
        double goalReachingThres;
        double carLongtitude; // ly

        double maxSteer; // [-maxSteer,maxSteer]
        double maxAccel;
        double minAccel; // [minAccel,maxAccel] can be negative
        bool isRearWheeled;
    };

    /**
     * @brief Parameters for global planner
     */
    struct ParamGlobal {
        double horizon;
        double car_width;
        double car_z_min;
        double car_z_max;
        double car_speed;
        double world_x_min;
        double world_y_min;
        double world_x_max;
        double world_y_max;
        double grid_resolution;
        double box_resolution;
        double box_max_size;
        bool has_wall;
        bool is_world_box_snu_frame;
    };

    struct ParamPredictor{
        int queueSize;
        float  zHeight;
        int polyOrder;
        double trackingTime; // it observation expires with this value, we detach predictor

    };

    /**
     * @brief Parameters for both planner.
     * @details This is retrieved from launch-param
     */
    struct Param {
        ParamGlobal g_param;
        ParamLocal l_param;
        ParamPredictor p_param;
    };
    /**
     * @brief State feed for MPC
     * @todo Conversion btw ROS messages / covariance
     */
    struct CarState{
        double x;
        double y;
        double v; //linear body velocity
        double theta;
    };

    struct ObstacleEllipse{
        Matrix<double,2,1> q;
        Matrix2d Q; // diag([1/r1^2 1/r2^2])
        double theta; // x-axis angle w.r.t x-axis of map
    };

    struct ObstaclePath{
        vector<ObstacleEllipse> obstPath;
    };

    struct ObstaclePathArray{
        vector<ObstaclePath> obstPathArray;
    };

    /**
     * @brief Input term of MPC
     */
    struct CarInput{
        double alpha; // linear acceleration
        double delta; // steering angle
    };

    /**
      * @brief Final output of MPC : trajectory of inputs.
      * @details We interpolate (ts,us,xs) to calculate current state or input
    */
    struct MPCResultTraj{
        vector<double> ts;
        vector<CarInput> us;
        vector<CarState> xs;
        CarState evalX(double t){
            vector<double> xSet(xs.size());
            vector<double> ySet(xs.size());
            // TDOO v, theta?
            for(uint i = 0 ; i < xs.size(); i++){
                xSet[i] = xs[i].x;
                ySet[i] = xs[i].y;
            }

            CarState xy;
            xy.x = interpolate(ts,xSet,t,true);
            xy.y = interpolate(ts,ySet,t,true);
            return xy;

        };
        CarInput evalU(double t){
            vector<double> aSet(us.size());
            vector<double> dSet(us.size());
            // TDOO v, theta?
            for(uint i = 0 ; i < us.size(); i++){
                aSet[i] = us[i].alpha;
                dSet[i] = us[i].delta;
            }

            CarInput input;
            input.alpha = interpolate(ts,aSet,t,true);
            input.delta = interpolate(ts,dSet,t,true);
            return input;
        }
    };

    /**
     * @brief The planning results and ingredients
     */
    class PlannerBase{

    private:
        // to be updated from callback
        shared_ptr<octomap::OcTree> octo_global_ptr;
        shared_ptr<octomap::OcTree> octo_local_ptr;

        CarState cur_state;
        SE3 cur_transform; // current tf of the car w.r.t the Tw0
        geometry_msgs::PoseStamped cur_pose; // current pose of the car w.r.t the Tw0
        CarState desired_state; //jungwon
        // to be updated by planners
        LanePath lane_path; //jungwon navigation planning output from Dabin Kim
        vector<Point> skeleton_path;
        vector<Corridor> corridor_seq;
        Corridor search_range;
        MPCResultTraj mpc_result;

        ObstaclePathArray obstaclePathArray;

    public:
        bool isGPsolved = false;
        bool isLPsolved = false;
        SE3 Tw0; // Referance frame of our node every incoming data should be transformed
        SE3 To0; // Transformation from  octomap ref frame to SNU
        SE3 T0s; // SNU to rotation of the first pose
        parser parse_tool;
        octomap_msgs::Octomap octomap_snu_msgs;

        // prediction module
        vector<Predictor::TargetManager> predictorSet; // TODO erase after indexed predictor
        Predictor::TargetManager predictorBase;
        list<Predictor::IndexedPredictor> indexedPredictorSet;

        // Get
        LanePath getLanePath() {return lane_path;};
        CarState getCarState() {return cur_state;};
        CarState getDesiredState() {return desired_state;}; //jungwon
        driving_msgs::VehicleCmd getCurInput(double t) {
            driving_msgs::VehicleCmd cmd;
            cmd.steer_angle_cmd =  mpc_result.evalU(t).delta;
            cmd.accel_decel_cmd = mpc_result.evalU(t).alpha;
            return cmd;
        };
        ObstaclePathArray getCurObstaclePathArray() {return obstaclePathArray;};
        vector<Point> getSkeletonPath() {return skeleton_path;}; //TODO: delete this after debugging
        vector<Corridor> getCorridorSeq() {return corridor_seq;};
        Corridor getSearchRange() {return search_range;};
        octomap::OcTree* getGlobalOctoPtr() {return octo_global_ptr.get();}
        octomap::OcTree* getLocalOctoPtr() {return octo_local_ptr.get();}
        MPCResultTraj getMPCResultTraj() {return mpc_result;}
        geometry_msgs::PoseStamped getCurPose() {return cur_pose;};
        SE3 getCurTf() {return cur_transform;};



        // Set from subscriber

        void setLaneWidth(double width) {
            lane_path.setWidth(width);
        }

        void setCarState(const CarState& carState_) { cur_state = carState_;};
        void setDesiredState(const CarState& desiredState_) {desired_state = desiredState_;};
        void setGlobalMap(octomap::OcTree* octoGlobalPtr_) {octo_global_ptr.reset(octoGlobalPtr_);};
        void setLocalMap(octomap::OcTree* octoLocalPtr_) {octo_local_ptr.reset(octoLocalPtr_);};
        void setCurPose(const geometry_msgs::PoseStamped poseStamped) {cur_pose = poseStamped;};
        void setCurTf(const SE3& T01) {cur_transform = T01;};
        // Set from planner
        void setLanePath(const LanePath& lane_path_in_) {lane_path = lane_path_in_;}
        void setSkeletonPath(const vector<Point>& skeleton_in_) {skeleton_path = skeleton_in_;}
        void setCorridorSeq(const vector<Corridor>& corridor_in_) {corridor_seq = corridor_in_;}
        void setSearchRange(const Corridor& search_range_in_) {search_range = search_range_in_;}
        void setMPCResultTraj(const MPCResultTraj& mpc_result_in_) {mpc_result = mpc_result_in_;}

        // Set from predictor
        // update obstaclePathArray so that it is prediction from [tPredictionStart,tPredictionStart+ horizon[
        // prepare prediction sequence for MPC
        void uploadPrediction(VectorXd tSeq_, double rNominal = 0) {
            VectorXf tSeq = tSeq_.cast<float>();
            // TODO predictor should be multiple
            obstaclePathArray.obstPathArray.clear();

            // ver0
            /**
            for(auto predictor : predictorSet){
                //predictor.update_predict(); // this will do nothing if observation is not enough
                if (predictor.is_prediction_available()) {
                    vector<geometry_msgs::Pose> obstFuturePose = predictor.eval_pose_seq(tSeq);
                    ObstaclePath obstPath;
                    for (auto &obstPose : obstFuturePose) {
                        // construct one obstaclePath
                        ObstacleEllipse obstE;
                        obstE.q = Vector2d(obstPose.position.x, obstPose.position.y);

                        // TODO shape should be consider later and its frame
                        obstE.Q.setZero();
                        obstE.Q(0, 0) = 1 / pow(rNominal, 2);
                        obstE.Q(1, 1) = 1 / pow(rNominal, 2);


                        obstPath.obstPath.push_back(obstE);
                    }
                    obstaclePathArray.obstPathArray.push_back(obstPath);
                }
            }
            **/
            // ver 1
            // TODO check the shape
            for(auto idPredictor : indexedPredictorSet){

                auto predictor = get<1>(idPredictor);
                //predictor.update_predict(); // this will do nothing if observation is not enough
                if (predictor.is_prediction_available()) {
                    vector<geometry_msgs::Pose> obstFuturePose = predictor.eval_pose_seq(tSeq);
                    ObstaclePath obstPath;
                    for (auto obstPose : obstFuturePose) {
                        // construct one obstaclePath
                        ObstacleEllipse obstE;
                        obstE.q = Vector2d(obstPose.position.x, obstPose.position.y);

                        // TODO exact tf should be handled / box to elliposid
                        obstE.Q.setZero();
                        obstE.Q(0, 0) = 1 / pow(predictor.getLastDimensions()(0)/sqrt(2), 2);
                        obstE.Q(1, 1) = 1 / pow(predictor.getLastDimensions()(1)/sqrt(2), 2);

                        obstPath.obstPath.push_back(obstE);
                    }
                    obstaclePathArray.obstPathArray.push_back(obstPath);
                }
            }


        }
    };
    /**
     * @brief Abstract class. The shared attributes to be inherited to the derived classes
     */
    class AbstractPlanner{

    protected:
        // Shared resource
        shared_ptr<PlannerBase> p_base; /**< This is the shared resource. Multiple thread will use this data. */

    public:
        AbstractPlanner(shared_ptr<PlannerBase> p_base_):p_base(p_base_) {};
        virtual bool plan(double t ) = 0;
        virtual bool isCurTrajFeasible() = 0;

    };

}


#endif //ATYPICAL_DRIVING_PLANNERBASE_H
