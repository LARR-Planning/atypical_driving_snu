//
// Created by jbs on 20. 5. 8..
//

#ifndef ATYPICAL_DRIVING_VECTORMAP_H
#define ATYPICAL_DRIVING_VECTORMAP_H

#include <vector>
#include <geometry_msgs/Point.h>

using namespace std;
using namespace geometry_msgs;


namespace Planner{
    struct LaneNode{
        vector<Point> laneCenters;
        double width;
    };

    struct LanePath{
        vector<LaneNode> lanes;
    };
}
#endif //ATYPICAL_DRIVING_VECTORMAP_H
