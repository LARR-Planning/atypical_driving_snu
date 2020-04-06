//
// Created by jbs on 20. 4. 6..
//

#ifndef ATYPICAL_DRIVING_PLANNERBASE_H
#define ATYPICAL_DRIVING_PLANNERBASE_H

#include <octomap/OcTree.h>
#include <vector>
#include <memory>
#include <geometry_msgs/Twist.h>

using namespace std;

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
    };

    /**
     * @brief Parameters for global planner
     */
    struct ParamGlobal {
        double horizon;
    };

    /**
     * @brief Parameters for both planner.
     * @details This is retrieved from launch-param
     */
    struct Param {
        ParamGlobal g_param;
        ParamLocal l_param;
    };
    /**
     * @brief State feed for MPC
     * @todo Conversion btw ROS messages / covariance
     */
    struct CarState{
        double x;
        double y;
        double v; // sign?
        double theta;
    };


    /**
     * @brief Input term of MPC
     */
    struct CarInput{
        double v; // linear velocity
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

        // to be updated by planners
        vector<Corridor> corridor_seq;
        MPCResultTraj mpc_result;

    public:
        // Get
        CarState getCarState() {return cur_state;};
        CarInput getCurInput() { return CarInput(); }; // do some interpolation

        vector<Corridor> getCorridorSeq() {return corridor_seq;};
        octomap::OcTree* getGlobalOctoPtr() {return octo_global_ptr.get();}
        octomap::OcTree* getLocalOctoPtr() {return octo_local_ptr.get();}

        // Set
        void setCarState(const CarState& carState_) { cur_state = carState_;};

        void setCorridorSeq(const vector<Corridor>& corridor_in_) {corridor_seq = corridor_in_;}
        void setMPCResultTraj(const MPCResultTraj& mpc_result_in_) {mpc_result = mpc_result_in_;}

    };
}


#endif //ATYPICAL_DRIVING_PLANNERBASE_H
