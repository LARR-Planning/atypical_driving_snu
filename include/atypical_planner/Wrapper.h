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

        /**
         *  Publisher
         */

        ros::Publisher pubPath; // publisher for result path

        /**
         * Subscriber
         */
        ros::Subscriber subCarPoseCov; /**< car state from KAIST */


        /**
         * Callback functions
         */

        void cbCarPoseCov(geometry_msgs::PoseWithCovarianceConstPtr dataPtr);

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
