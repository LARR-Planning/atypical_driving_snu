#ifndef CONX_HPP
#define CONX_HPP

#include <Eigen/Dense>
#include <optimization_module/dimension.h>
#include <optimization_module/parameters/dyn_parameter.h>

using namespace Eigen;
namespace symbolic_functions
{
    Matrix<double,Nc,Nx> conx(Matrix<double,Nx,1> x_,Matrix<double,Nu,1> u,Matrix<double, 4, 1> sfc_modified_temp)
    {
        float x = x_(0,0);
        float y = x_(1,0);
        float v = x_(2,0);
        float del = x_(3,0);
        float th = x_(4,0);

        float a = u(0,0);
        float deldot = u(1,0);

        Matrix<double,Nc,Nx> CONX;
        CONX.setZero();
        CONX(0,3) = 1.0; //steering angle min
        CONX(1,3) = -1.0; //steering angle max
        CONX(4,0) = 1.0; // sfc xl
        CONX(5,0) = -1.0; // sfc xu
        CONX(6,1) = 1.0; // sfc yl
        CONX(7,1) = -1.0; //sfc yu
        return CONX;
    }
}




#endif