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
using namespace Eigen;
double interpolate(vector<double> &xData, vector<double> &yData, double x, bool extrapolate );
Vector3d applyTransform(const SE3& Tab, const Vector3d & vb);




#endif //ATYPICAL_DRIVING_UTILS_H


