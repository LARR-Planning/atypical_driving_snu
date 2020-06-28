#ifndef CON_EXP_HPP
#define CON_EXP_HPP

#include <Eigen/Dense>
#include <optimization_module/dimension.h>
#include <optimization_module/parameters/dyn_parameter.h>

using namespace Eigen;
namespace symbolic_functions
{
    Matrix<double,Nc,1> con_exp(Matrix<double,Nx,1> x_, Matrix<double,Nu,1>u,
            Matrix<double,4,1> sfc_modified)
    {
        float x = x_(0,0);
        float y = x_(1,0);
        float v = x_(2,0);
        float a = x_(3,0);
        float del = x_(4,0);
        float th = x_(5,0);

        float adot = u(0,0);
        float deldot = u(1,0);

        Matrix<double,Nc,1> CON;
        CON(0,0) = x - (sfc_modified(0,0)-dist_minus);
        CON(1,0) = (sfc_modified(1,0)+dist_minus)-x;
        CON(2,0) = y - (sfc_modified(2,0)-dist_minus);
        CON(3,0) = (sfc_modified(3,0)+dist_minus)-y;
        CON(4,0) = a - acc_min;
        CON(5,0) = acc_max - a;
        CON(6,0) = del - steer_min;
        CON(7,0) = steer_max - del;

        CON(8,0) = adot - jerk_min;
        CON(9,0) = jerk_max - adot;
        CON(10,0) = deldot - steer_dot_min;
        CON(11,0) = steer_dot_max - deldot;



        return CON;
    }
}
#endif
