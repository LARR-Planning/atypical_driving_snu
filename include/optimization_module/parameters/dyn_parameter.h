#ifndef DYN_PARAMETER_H
#define DYN_PARAMETER_H
#include <Eigen/Dense>

using namespace Eigen;



const double car_width = 2.0;
const double car_length = 0.0;
const double invL = 0.43103448;
const double dist_safe = 10;
const double inv_dist_safe = 0.1;
const double steer_min = - 0.785; //-30 deg
const double steer_max = 0.785; // 30 deg
const double acc_min = -1.5;
const double acc_max = 1.5;

#endif
