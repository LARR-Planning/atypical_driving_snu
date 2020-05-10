#include <atypical_planner/Wrapper.h>
#include <third_party/Vectormap.h>
#include "nav_msgs/Odometry.h"


#include "Eigen/Core"


using namespace Planner;
using namespace Eigen;

int main(int argc, char ** argv){
    // This code mock the vector map cases

    LaneNode l1,l2;
    int N1 = 20, N2 = 10;

    l1.width = 10;
    l2.width = 13;

    VectorXf l1X(N1); l1X.setZero();
    VectorXf l1Y(N1); l1Y.setLinSpaced(N1,0,73);
    VectorXf l2X(N2); l2X.setLinSpaced(N2,0,49);
    VectorXf l2Y(N2); l2Y.setConstant(73);

    for (int n = 0 ; n < N1 ; n++){
        geometry_msgs::Point pnt;
        pnt.x = l1X(n);
        pnt.y = l1Y(n);
        l1.laneCenters.push_back(pnt);
    }

    for (int n = 0 ; n < N2 ; n++){
        geometry_msgs::Point pnt;
        pnt.x = l2X(n);
        pnt.y = l2Y(n);
        l2.laneCenters.push_back(pnt);
    }








}


