//
// Created by jbs on 20. 4. 6..
//

#ifndef ATYPICAL_DRIVING_WRAPPER_H
#define ATYPICAL_DRIVING_WRAPPER_H


#include <atypical_planner/LocalPlanner.h>
#include <atypical_planner/GlobalPlanner.h>

#include <ros/ros.h>

namespace Planner{

    /**
     * @brief ROS communicator such as topic advertising/subscription and ros-param parsing.
     */
    class RosWrapper{

    private:
        PlannerBase* p_base; // planning data

        // node handle
        ros::NodeHandle nh;

        // publisher

        // subscriber

    public:
        RosWrapper();
        void updateParam(Param& param_);
        void publish();
    };


    class Wrapper{

    private:
        shared_ptr<PlannerBase> p_base_shared;
        Param param;
        LocalPlanner* lp_ptr;
        GlobalPlanner* gp_ptr;
        RosWrapper ros_wrapper;

        void update();
        void planGlobal();
        void planLocal();

    public:
        Wrapper();
        void run();
    };







}




#endif //ATYPICAL_DRIVING_WRAPPER_H
