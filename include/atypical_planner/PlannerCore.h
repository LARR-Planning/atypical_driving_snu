//
// Created by jbs on 20. 4. 6..
//

#ifndef ATYPICAL_DRIVING_PLANNERBASE_H
#define ATYPICAL_DRIVING_PLANNERBASE_H

#include <vector>
#include <queue>
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

#include <geometry_msgs/TwistStamped.h>

using namespace std;
using namespace Eigen;

namespace Planner {

    const int OCCUPANCY = 60;

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
        double lxFrontWheel;
        double maxSteer; // [-maxSteer,maxSteer]
        double maxAccel;
        double minAccel; // [minAccel,maxAccel] can be negative
        double nominal_speed;
        double sameWptsDistance; //Criteria for regard as same position
        int sameNumPointStopping;
        bool isRearWheeled;
        double sfcMargin;
        double period;
        int N_corr;
	    double dynObstRange;
        //ilQR parameters
        Matrix<double,6,1> state_weight;
        Matrix<double,6,1> final_weight;
        Matrix<double,2,1> input_weight;
    };

    /**
     * @brief Parameters for global planner
     */
    struct ParamGlobal {
        // JBS for map representation

        double pcl_lx;
        double pcl_ly;
        double pcl_z_min;
        double pcl_z_max;
        int pcl_dbscan_minpnts;
        double pcl_dbscan_eps;
        double ransac_ground_offest;
        double ransac_distance_threshold;
        bool use_ransac;

//        double horizon;
        double car_width;
//        double car_z_min;
//        double car_z_max;
        // by JBS
        double car_speed_max;
        double car_speed_min;
        double curvature_thres;
        bool use_keti_velocity = false;

//        double car_acceleration;
//        double world_x_min;
//        double world_y_min;
//        double world_x_max;
//        double world_y_max;
        double grid_resolution;
//        double box_resolution;
//        double box_max_size;
//        bool has_wall;
//        bool is_world_box_snu_frame;
        double period;
//        int max_smoothing_iteration;
//        double smoothing_margin;
        double smoothing_distance;
        double smoothing_cliff_min_bias;
        double smoothing_cliff_ratio;
        double start_smoothing_distance;
        double max_steering_angle;
        double corridor_width_min;
        double corridor_width_dynamic_min;
        double corridor_max_bias;
        double safe_distance;
        double v_ref_past_weight;
        double nominal_acceleration;
        double object_velocity_threshold;
        double max_obstacle_prediction_query_size;
        double acc_stop_distance;
        double acc_stop_angle;

        bool use_static_object;
        bool use_lane_point_first;
        int smoothing_cliff_n_check_idx;
        double blocked_by_object_distance;
    };

    struct ParamPredictor{
        int queueSize;
        float  zHeight;
        int polyOrder;
        double trackingTime; // it observation expires with this value, we detach predictor
        string log_dir; // base dir for log
        double staticCriteria; // bigger than this = dynamic
        bool predictWithKeti;
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
    // (x-q)'Q(x-q) >= 1 : safe region
    struct ObstacleEllipse{
        Matrix<double,2,1> q;
        Matrix2d Q; // diag([1/r1^2 1/r2^2])
        double r1;
        double r2;
        double theta; // x-axis angle w.r.t x-axis of SNU
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct ObstaclePath{
        Vector2d constantVelocityXY; // from linear fitting (orignal method)
        Vector2d meanVelocity; // from  direct KETI. (averaged)
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
        visualization_msgs::MarkerArray getMPC(const string& frame_id);
        bool isSuccess;
    };

    /**
     * @brief Initial lane
     */
    struct Lane{
        vector<Vector2d, aligned_allocator<Vector2d>> points;
        vector<double> widths;
        Lane(){};
        Lane(const LanePath& lanePath);
        void untilGoal(double goal_x,double goal_y);
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
        Vector2d lanePoint;
        Vector2d rightPoint;
        Vector2d rightBoundaryPoint;
        bool isNearObject;
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
        bool isBlocked;
        bool isBlockedByObject;

        int n_total_markers = 0;

        Vector2d evalX(const std::vector<Vector2d, aligned_allocator<Vector2d>>& points_vector, double t);
        Vector2d evalX(double t);
        Vector2d evalSidePoint(double t, bool isLeft);
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
        nav_msgs::OccupancyGrid localMapBuffer;


        // Flag
        bool isGPsolved = false;
        bool isLPsolved = false;
        bool isReached = false;
        bool isLPPassed = false;
        // Lane
        parser parse_tool;
        LanePath lane_path; // deprecated (better not to be used in routine source block )
        Lane laneOrig;
        Lane laneSliced; // lane in current sliding window
        SkeletonLane laneSkeleton;
        SmoothLane laneSmooth;
        double laneSpeed = 0 ; // current speed to be applied to the lane in slice
        double laneCurvature;  // current mean curvature of the slide

        int smoothing_type; // 0:exponential average, 1: moving average(handle), 2: ignore little handling
        double stopSpeed;
        double weight_smooth; // exponential weight smoothing
        double ignore_angle; // Ignore small angle range
        int smooth_horizon; // moving average smoothing

        vector<Corridor> corridor_seq;
        Corridor search_range;
        MPCResultTraj mpc_result;
        driving_msgs::VehicleCmd ctrl_previous;
        vector<driving_msgs::VehicleCmd> ctrl_history;
        ObstaclePathArray obstaclePathArray;

        CarState cur_state;
        mutex mSet[2]; /**< two mutex. 0 = planning vs subscription & 1 = update vs publishing */

        // Transformation
        SE3 Tws; // global UTM to SNU (static)
        SE3 Tso; // occupancy map frame to SNU
        SE3 Tsb; // SNU to body



        TwistStamped lastPublishedInput;
        string log_file_name_base;
        geometry_msgs::PoseStamped cur_pose; // current pose of the car w.r.t the {SNU}
        deque<CarState> desired_state_seq;

        double goal_thres;
        double goal_x;
        double goal_y;
        Vector3d goalXYSNU;

        int flag = 0;

        int count_iter =0;
        bool isOccupied(Vector2d queryPoint); // query point frame = SNU
        bool isOccupied(Vector2d queryPoint1, Vector2d queryPoint2); // rayIntersection query point frame = SNU

        bool isObject(const Vector2d& queryPoint, int maxObstQuerySize, double& min_distance_to_object, double& velocity, bool use_keti_vel = false); // query point frame = SNU


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
        driving_msgs::VehicleCmd getCurInput(double t);
//        driving_msgs::VehicleCmd getCurInput(double t) {
//            driving_msgs::VehicleCmd cmd;
//            cmd.steer_angle_cmd =  mpc_result.evalU(t).delta;
//            cmd.accel_decel_cmd = mpc_result.evalU(t).alpha;
//            return cmd;
//        };

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
