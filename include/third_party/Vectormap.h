//
// Created by jbs on 20. 5. 8..
//

#ifndef ATYPICAL_DRIVING_VECTORMAP_H
#define ATYPICAL_DRIVING_VECTORMAP_H

#include <vector>
#include <geometry_msgs/Point.h>
#include <Eigen/Core>
#include <nav_msgs/Path.h>
#include <third_party/Utils.h>

using namespace std;
using namespace geometry_msgs;
// using namespace Eigen;

namespace Planner{

    // Lane node and path are deprecated ...
    struct LaneNode{
        vector<Point> laneCenters;
        double width;
    };

    struct LanePath{
        vector<LaneNode> lanes;
        void applyTransform(const Eigen::Matrix4d& Tab){
            for (auto & lane :lanes){
                for(auto it = lane.laneCenters.begin() ; it!= lane.laneCenters.end(); it++){

                       Eigen::Vector4d xb(it->x,it->y,0,1);
                       Eigen::Vector4d xa = Tab*xb;
                       it->x = xa(0);
                       it->y = xa(1);
                }
            }
        }

        void applyTransform(const SE3& Tab){
            for (auto & lane :lanes){
                for(auto it = lane.laneCenters.begin() ; it!= lane.laneCenters.end(); it++){
                    Eigen::Vector3d xb(it->x,it->y,0);
                    Eigen::Vector3d xa = Tab*xb;
                    it->x = xa(0);
                    it->y = xa(1);
                }
            }
        }

        nav_msgs::Path getPath(string frame_id){
            nav_msgs::Path nodeNavPath;
            nodeNavPath.header.frame_id = frame_id.c_str();
            for (auto & lane :lanes){
                for(auto it = lane.laneCenters.begin() ; it!= lane.laneCenters.end(); it++){
                    geometry_msgs::PoseStamped aPoint;
                    aPoint.pose.position.x = it->x;
                    aPoint.pose.position.y = it->y;
                    nodeNavPath.poses.push_back(aPoint);
                }
            }
            return nodeNavPath;
        }

        void setWidth(double width){
            for (auto & lane :lanes){
                    lane.width = width;
                }
            }


    };
}
#endif //ATYPICAL_DRIVING_VECTORMAP_H
