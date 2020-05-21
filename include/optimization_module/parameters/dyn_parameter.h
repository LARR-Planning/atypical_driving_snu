#ifndef DYN_PARAMETER_H
#define DYN_PARAMETER_H
#include <Eigen/Dense>

using namespace Eigen;




const double invL = 0.37037037037;
const double dist_safe = 10;
const double inv_dist_safe = 0.1;
const double steer_min = - 0.5; //-30 deg
const double steer_max = 0.5; // 30 deg
const double acc_min = -3;
const double acc_max = 1;

#endif