#include <atypical_planner/Wrapper.h>


int main(int argc,char** argv) {
    ros::init(argc,argv,"atypical_planning_test");
    Planner::Wrapper snu_driving_wrapper;
    snu_driving_wrapper.run();

    return 0;
}
