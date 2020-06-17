//
// Created by jbs on 20. 4. 6..
//

#ifndef ATYPICAL_DRIVING_PLANNERBASE_H
#define ATYPICAL_DRIVING_PLANNERBASE_H

#include <vector>
#include <memory>
#include <mutex>
#include <thread>
#include <string>
#include <queue>
#include <deque>
#include <cmath>
#include <list>
#include <tuple>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <fstream>

#include <third_party/Vectormap.h>
#include <third_party/Utils.h>
#include <third_party/parser.h>
#include <third_party/Prediction/target_manager.hpp>

#include <driving_msgs/VehicleCmd.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>

#include <occupancy_grid_utils/coordinate_conversions.h>
#include <occupancy_grid_utils/ray_tracer.h>


using namespace std;
using namespace Eigen;

namespace Planner {

    const int OCCUPANCY = 70;

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
        VectorXf getPretty();
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
        double nominal_speed;
        bool isRearWheeled;
        double period;

        //ilQR parameters
        Matrix<double,5,1> state_weight;
        Matrix<double,5,1> final_weight;
        Matrix<double,2,1> input_weight;
    };

    /**
     * @brief Parameters for global planner
     */
    struct ParamGlobal {
        double horizon;
        double car_width;
        double car_z_min;
        double car_z_max;
        // by JBS
        double car_speed_max;
        double car_speed_min;
        double curvature_thres;

        double car_acceleration;
        double world_x_min;
        double world_y_min;
        double world_x_max;
        double world_y_max;
        double grid_resolution;
        double box_resolution;
        double box_max_size;
        bool has_wall;
        bool is_world_box_snu_frame;
        double period;
        int max_smoothing_iteration;
        double smoothing_margin;
        double max_steering_angle;
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

        void print(){
            printf("Current car state: x = %f, y = %f, v = %f, theta = %f\n ",x,y,v,theta);
        }
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
        MatrixXf getPretty(double t_stamp);
        CarState evalX(double t);
        CarInput evalU(double t);
    };


    /**
     * @brief Initial lane
     */
    struct Lane{
        vector<Vector2d, aligned_allocator<Vector2d>> points;
        vector<double> widths;
        Lane(){};
        Lane(const LanePath& lanePath);
        vector<Vector2d, aligned_allocator<Vector2d>> slicing(const CarState& curCarState,Vector2d windowOrig,double w, double h , int & startIdx , int & endIdx );
        nav_msgs::Path getPath(string frame_id);
        visualization_msgs::MarkerArray getSidePath(string frame_id);
    };

    /**
     * @brief Modified lane by global planner
     */
    struct SkeletonLane : public Lane{

    };

    /**
     * @brief Modified lane by global planner
     */
    class LaneTreeElement{
    public:
        LaneTreeElement(){
            id = -1;
            distance = 0;
            total_width = 0;
            visited = false;
            next = -1;
        }
        int id;
        Vector2d leftBoundaryPoint;
        Vector2d leftPoint;
        Vector2d midPoint;
        Vector2d rightPoint;
        Vector2d rightBoundaryPoint;
        vector<int> children;

        int distance;
        double total_width;
        double width;

        bool visited;
        int next;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    /**
     * @brief final output
     */
    struct SmoothLane: public Lane{
        vector<double> ts;
        vector<double> box_size;
        vector<Vector2d, aligned_allocator<Vector2d>> leftBoundaryPoints;
        vector<Vector2d, aligned_allocator<Vector2d>> rightBoundaryPoints;

        int n_total_markers = 0;

        Vector2d evalX(const vector<Vector2d, aligned_allocator<Vector2d>>& points, double t);
        double evalWidth(double t);
        visualization_msgs::MarkerArray getPoints(const string& frame_id);
    };

    /**
     * @brief The planning results and ingredients
     */
    class PlannerBase{

    public:
        // Map
        nav_msgs::OccupancyGrid localMap;

        // Flag
        bool isGPsolved = false;
        bool isLPsolved = false;

        // Lane
        parser parse_tool;
        LanePath lane_path; // deprecated (better not to be used in routine source block )
        Lane laneOrig;
        Lane laneSliced; // lane in current sliding window
        SkeletonLane laneSkeleton;
        SmoothLane laneSmooth;
        double laneSpeed; // current speed to be applied to the lane in slice
        double laneCurvature;  // current mean curvature of the slide

        vector<Corridor> corridor_seq;
        Corridor search_range;
        MPCResultTraj mpc_result;

        ObstaclePathArray obstaclePathArray;

        CarState cur_state;
        mutex mSet[2]; /**< two mutex. 0 = planning vs subscription & 1 = update vs publishing */

        // Transformation
        SE3 Tws; // global UTM to SNU (static)
        SE3 Tso; // occupancy map frame to SNU
        SE3 Tsb; // SNU to body


        string log_file_name_base;
        geometry_msgs::PoseStamped cur_pose; // current pose of the car w.r.t the {SNU}
        deque<CarState> desired_state_seq;

        double goal_thres;

        bool isOccupied(Vector2d queryPoint); // query point frame = SNU
        bool isOccupied(Vector2d queryPoint1, Vector2d queryPoint2); // rayIntersection query point frame = SNU

        // Corridor generation
        Corridor expandCorridor(Vector2d point, Vector2d leftBoundaryPoint, Vector2d rightBoundaryPoint, double max_box_size, double map_resolution);
        std::vector<Corridor> expandCorridors(std::vector<double> ts, double map_resolution);
        visualization_msgs::MarkerArray getTestCorridors(std::string frame_id); //TODO: debug purpose, delete this after debugging - jungwon

        // prediction module
        vector<Predictor::TargetManager> predictorSet; // TODO erase after indexed predictor
        Predictor::TargetManager predictorBase;
        list<Predictor::IndexedPredictor> indexedPredictorSet;

        // Get
        CarState getCarState() {return cur_state;};
        LanePath getLanePath() {return lane_path;};


        CarState getDesiredState() {return desired_state_seq.front();}; //jungwon
        bool isGoalReach() {
            double dist = sqrt(pow(getDesiredState().x - cur_state.x,2) +
            pow(getDesiredState().y - cur_state.y,2));
            return (dist < goal_thres);
        }

        driving_msgs::VehicleCmd getCurInput(double t) {
            driving_msgs::VehicleCmd cmd;
            cmd.steer_angle_cmd =  mpc_result.evalU(t).delta;
            cmd.accel_decel_cmd = mpc_result.evalU(t).alpha;
            return cmd;
        };
        ObstaclePathArray getCurObstaclePathArray() {return obstaclePathArray;};
        vector<Corridor> getCorridorSeq() {return corridor_seq;}; // Returns just all the latest
        vector<Corridor> getCorridorSeq(double t0,double tf ); // start time is set to zero

        Corridor getSearchRange() {return search_range;};
        MPCResultTraj getMPCResultTraj() {return mpc_result;}
        geometry_msgs::PoseStamped getCurPose() {return cur_pose;};

        // Set from subscriber
        void setLanePath(const LanePath& lane_path_in_) {lane_path = lane_path_in_;}
        void setCarState(const CarState& carState_) { cur_state = carState_;};
        void setCurPose(const geometry_msgs::PoseStamped poseStamped) {cur_pose = poseStamped;};
        // Set from planner
        void setCorridorSeq(const vector<Corridor>& corridor_in_) {corridor_seq = corridor_in_;}
        void setSearchRange(const Corridor& search_range_in_) {search_range = search_range_in_;}
        void setMPCResultTraj(const MPCResultTraj& mpc_result_in_) {mpc_result = mpc_result_in_;}

        // prepare prediction sequence for MPC
        void uploadPrediction(VectorXd tSeq_, double rNominal = 0);

        // logger
        void log_state_input(double t_cur);
        void log_corridor(double t_cur,double tf);
        void log_mpc(double t_cur);

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
