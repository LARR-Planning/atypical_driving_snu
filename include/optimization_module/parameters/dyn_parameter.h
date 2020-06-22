#ifndef DYN_PARAMETER_H
#define DYN_PARAMETER_H
#include <Eigen/Dense>

using namespace Eigen;

const double car_width = 0.0;
const double car_length = 0.0;
const double invL = 0.37037;
const double dist_safe = 100;
const double inv_dist_safe = 0.1;
const double steer_min = -0.52; //-30 deg
const double steer_max = 0.52; // 30 deg
const double acc_min = -3;
const double acc_max = 1;
const double jerk_min = -10;
const double jerk_max = 10;
const double steer_dot_min = -5;
const double steer_dot_max = 5;





#endif
