//
// Created by jbs on 20. 5. 14..
//

#ifndef ATYPICAL_DRIVING_UTILS_H
#define ATYPICAL_DRIVING_UTILS_H

#include <vector>
#include <geometry_msgs/Pose.h>
#include <Eigen/Geometry>

typedef  Eigen::Transform<double,3,Eigen::Affine> SE3;
using namespace std;
double interpolate(vector<double> &xData, vector<double> &yData, double x, bool extrapolate );


#endif //ATYPICAL_DRIVING_UTILS_H


