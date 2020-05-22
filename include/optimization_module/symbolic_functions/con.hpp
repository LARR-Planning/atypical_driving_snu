#ifndef CON_HPP
#define CON_HPP

#include <Eigen/Dense>
#include <optimization_module/dimension.h>
#include <optimization_module/parameters/dyn_parameter.h>

using namespace Eigen;
namespace symbolic_functions
{
    Matrix<double,Nc,1> con(Matrix<double,Nx,1> x_, Matrix<double,Nu,1>u, Matrix<double, 4, 1> sfc_modified_temp)
    {
        float x = x_(0,0);
        float y = x_(1,0);
        float v = x_(2,0);
        float del = x_(3,0);
        float th = x_(4,0);

        float a = u(0,0);
        float deldot = u(1,0);

        Matrix<double,Nc,1> CON;
        CON(0,0) = del -steer_min;
        CON(1,0) = steer_min - del;
        CON(2,0) = a - acc_min;
        CON(3,0) = acc_max - a;
        CON(4,0) = x - (sfc_modified_temp(0,0) + car_length*0.5);
        CON(5,0) = (sfc_modified_temp(1,0) - car_length*0.5)-x;
        CON(6,0) = y - (sfc_modified_temp(2,0) + car_length*0.5);
        CON(7,0) = (sfc_modified_temp(3,0) - car_length*0.5)-y;


        return CON;
    }
}
#endif
