#ifndef COSTU_HPP
#define COSTU_HPP

#include <Eigen/Dense>
#include <optimization_module/dimension.h>
#include <optimization_module/parameters/dyn_parameter.h>

using namespace Eigen;
namespace symbolic_functions
{
    Matrix<double,Nu,1> costu(Matrix<double,Nx,1>x_,
                Matrix<double,Nu,1>u_,
                Matrix<double,Nx,1>Q,
                Matrix<double,Nu,1>R,
                Matrix<double,2,1>g,
                Matrix<double,2,1>obs)
    {
        // float x = x_(0,0);
        // float y = x_(1,0);
        // float v = x_(2,0);
        // float del = x_(3,0);
        // float th = x_(4,0);
        float a = u_(0,0);
        float deldot = u_(1,0);
        // float Q1 = Q(0);
        // float Q2 = Q(1);
        float R1 = R(0);
        float R2 = R(1);
        Matrix<double,Nu,1> A0;
        A0(0,0) = R1*a;
        A0(1,0) = R2*deldot;
        return A0;
    }
}


#endif