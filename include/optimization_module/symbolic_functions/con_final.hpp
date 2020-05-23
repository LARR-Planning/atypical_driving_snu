#ifndef CON_FINAL_HPP
#define CON_FINAL_HPP

#include <Eigen/Dense>
#include <optimization_module/dimension.h>
#include <optimization_module/parameters/dyn_parameter.h>

using namespace Eigen;
namespace symbolic_functions
{
    Matrix<double,Nc,1> con_final(Matrix<double,Nx,1> x_, Matrix<double,Nu,1>u,Matrix<double, 4, 1> sfc_modified_temp)
    {
        float x = x_(0,0);
        float y = x_(1,0);
        float v = x_(2,0);
        float del = x_(3,0);
        float th = x_(4,0);

        float a = u(0,0);
        float deldot = u(1,0);

        Matrix<double,Nc,1> CON_FINAL;
        CON_FINAL.setZero();
        CON_FINAL(0,0) = del -steer_min;
        CON_FINAL(1,0) = steer_min - del;
        CON_FINAL(4,0) = x - (sfc_modified_temp(0,0) + car_width*0.5);
        CON_FINAL(5,0) = (sfc_modified_temp(1,0) - car_width*0.5)-x;
        CON_FINAL(6,0) = y - (sfc_modified_temp(2,0) + car_width*0.5);
        CON_FINAL(7,0) = (sfc_modified_temp(3,0) -car_width*0.5)-y;

        CON_FINAL(8,0) = x+car_length*cos(th) - (sfc_modified_temp(0,0) + car_width*0.5);
        CON_FINAL(9,0) = (sfc_modified_temp(1,0) - car_width*0.5)-x-car_length*cos(th);
        CON_FINAL(10,0) = y+car_length*sin(th) - (sfc_modified_temp(2,0) + car_width*0.5);
        CON_FINAL(11,0) = (sfc_modified_temp(3,0) - car_width*0.5)-y-car_length*sin(th);

        return CON_FINAL;
    }
}
#endif