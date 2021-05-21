#include <atypical_planner/Wrapper.h>


int main(int argc,char** argv) {
    ros::init(argc,argv,"atypical_planning_test");
    auto p_base_shared  = (make_shared<Planner::PlannerBase>());
    Planner::RosWrapper mapper(p_base_shared);
    Planner::Param param;
    mapper.updateParam(param);
    mapper.runROS();
    return 0;
}
