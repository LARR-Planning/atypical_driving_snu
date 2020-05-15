//
// Created by jbs on 20. 4. 6..
//

#ifndef ATYPICAL_DRIVING_WRAPPER_H
#define ATYPICAL_DRIVING_WRAPPER_H


#include <atypical_planner/LocalPlanner.h>
#include <atypical_planner/GlobalPlanner.h>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

namespace Planner{

    /**
     * @brief ROS communicator such as topic advertising/subscription and ros-param parsing.
     */
    class RosWrapper{

    private:
        double t0; // update at the constructor
        /**
         * Shared resource with other thread
         */
        shared_ptr<PlannerBase> p_base; // planning data
        mutex* mSet; /**< mSet[0] = locking btw subset of callbacks and plan() of planner  */

        // node handle
        ros::NodeHandle nh;
        bool isGlobalMapReceived = false;
        bool isLocalMapReceived = false;
        bool isCarPoseCovReceived = false;

        /**
         * Parameters
         */

        string worldFrameId;
        int max_marker_id; //count current published markers

        /**
         * Topic to be published
         */
        // topics to be published
        nav_msgs::Path planningPath; // can be obtained from mpcResultTraj
        visualization_msgs::MarkerArray corridorSeq;
        visualization_msgs::MarkerArray obstaclePrediction;

        /**
         *  Publisher
         */

        ros::Publisher pubPath; // publisher for result path
        ros::Publisher pubCorridorSeq; // publisher for current corridor sequence
        ros::Publisher pubObservationMarker; // publisher for observed position for obstacles
        ros::Publisher pubPredictionArray; // publisher for prediction of the target
        ros::Publisher pubCurCmd; // if MPC has been solved, it emits the command

        /**
         * Subscriber
         */
        ros::Subscriber subCarPoseCov; /**< car state from KAIST */
        ros::Subscriber subDesiredCarPose; // desired pose from user
        ros::Subscriber subGlobalMap; // global map from ????
        ros::Subscriber subLocalMap; // local map from LIDAR????

        ros::Subscriber subExampleObstaclePose; //

        /**
         * Callback functions
         */

        void cbCarPoseCov(geometry_msgs::PoseWithCovarianceConstPtr dataPtr);
        void cbDesiredCarPose(geometry_msgs::PoseConstPtr dataPtr);
        void cbGlobalMap(const octomap_msgs::Octomap& octomap_msg);
        void cbLocalMap(const octomap_msgs::Octomap& octomap_msg);

        // TODO currently we only receive the location of obstacles not with shape
        void cbObstacles(const geometry_msgs::PoseStamped& obstPose);

        /**
         * Core routines in while loop of ROS thread
         */
        void publish();
        void prepareROSmsgs();

    public:
        RosWrapper(shared_ptr<PlannerBase> p_base_,mutex* mSet_);
        void updateParam(Param& param_);
        void updatePrediction();
        void runROS();
        bool isAllInputReceived();
        double curTime() {return (ros::Time::now().toSec()-t0);};
    };

    /**
     * @brief Wrapping module for RosWrapper and the two planners.
     */
    class Wrapper{

    private:
        shared_ptr<PlannerBase> p_base_shared;  /**< pointer for shared resource  */
        mutex mSet[2]; /**< two mutex. 0 = planning vs subscription & 1 = update vs publishing */

        thread threadPlanner;
        thread threadRosWrapper;

        Param param; /**< global and local planner parameters  */

        LocalPlanner* lp_ptr; /**< local planner */
        GlobalPlanner* gp_ptr; /**< global planner */
        RosWrapper* ros_wrapper_ptr; /**< ros wrapper */

        // Do two-staged planning
        bool plan(double tTrigger); // this includes the two below routines
        void updateCorrToBase(); // update the results of the planners to p_base
        void updateMPCToBase(); // update the results of the planners to p_base
        void updatePrediction();
        void runPlanning(); // planning thread.
    public:
        Wrapper();
        void run();
    };
}




#endif //ATYPICAL_DRIVING_WRAPPER_H
