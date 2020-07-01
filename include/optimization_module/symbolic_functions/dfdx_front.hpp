#ifndef DFDX_FRONT_HPP
#define DFDX_FRONT_HPP

#include <Eigen/Dense>
#include <optimization_module/dimension.h>
#include <optimization_module/parameters/dyn_parameter.h>

using namespace Eigen;
namespace symbolic_functions
{
    Matrix<double,Nx,Nx> dfdx_front(Matrix<double,Nx,1> x_, Matrix<double,Nu,1>u)
    {
        float x = x_(0,0);
        float y = x_(1,0);
        float v = x_(2,0);
        float a = x_(3,0);
        float del = x_(4,0);
        float th = x_(5,0);
        float adot = u(0,0);
        float deldot = u(1,0);

        Matrix<double,Nx,Nx> linA;
        linA.setZero();
        linA(0,2) = std::cos(th)*cos(del);
        linA(0,4) = -v * std::cos(th)*sin(del);
        linA(0,5) = -v*sin(th)*cos(del);
        linA(1,2) = std::sin(th)*cos(del);
        linA(1,4) = -v*sin(th)*sin(del);
        linA(1,5) = v*cos(th)*cos(del);
        linA(2,3) = 1.0;
        linA(5,2) = std::sin(del)*invL;
        linA(5,4) = v*invL*cos(del);

        return linA;
    }
}



#endif