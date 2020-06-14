//
// Created by jbs on 20. 5. 14..
//

#ifndef ATYPICAL_DRIVING_UTILS_H
#define ATYPICAL_DRIVING_UTILS_H

#include <vector>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <Eigen/Geometry>


typedef  Eigen::Transform<double,3,Eigen::Affine> SE3;
using namespace std;
using namespace Eigen;

double interpolate(vector<double> &xData, vector<double> &yData, double x, bool extrapolate );
Vector3d applyTransform(const SE3& Tab, const Vector3d & vb);
using namespace geometry_msgs;

bool intersect(Point i0, Point i1, Point j0, Point j1);
int ccw(Point a, Point b, Point c);
nav_msgs::Path getPath(const vector<Vector2d> & point2dSeq,string frame_id);
double meanCurvature(const vector<Vector2d> & point2dSeq);
#endif //ATYPICAL_DRIVING_UTILS_H


