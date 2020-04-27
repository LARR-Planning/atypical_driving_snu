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

        /**
         * Topic to be published
         */
        // topics to be published
        nav_msgs::Path planningPath; // can be obtained from mpcResultTraj
        visualization_msgs::MarkerArray corridorSeq;

        /**
         *  Publisher
         */

        ros::Publisher pubPath; // publisher for result path
        ros::Publisher pubCorridorSeq; // publisher for current corridor sequence

        /**
         * Subscriber
         */
        ros::Subscriber subCarPoseCov; /**< car state from KAIST */
        ros::Subscriber subDesiredCarPose; // desired pose from user
        ros::Subscriber subGlobalMap; // global map from ????

        /**
         * Callback functions
         */

        void cbCarPoseCov(geometry_msgs::PoseWithCovarianceConstPtr dataPtr);
        void cbDesiredCarPose(geometry_msgs::PoseConstPtr dataPtr);
        void cbGlobalMap(const octomap_msgs::Octomap& octomap_msg);

        /**
         * Core routines in while loop of ROS thread
         */
        void publish();
        void prepareROSmsgs();


    public:
        RosWrapper(shared_ptr<PlannerBase> p_base_,mutex* mSet_);
        void updateParam(Param& param_);
        void runROS();
        bool isAllInputReceived();
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
        bool plan(); // this includes the two below routines
        void updateToBase(); // update the results of the planners to p_base

        void runPlanning(); // planning thread.

    public:
        Wrapper();
        void run();
    };





}




#endif //ATYPICAL_DRIVING_WRAPPER_H