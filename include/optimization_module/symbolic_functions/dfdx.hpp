#ifndef DFDX_HPP
#define DFDX_HPP

#include <Eigen/Dense>
#include <optimization_module/dimension.h>
#include <optimization_module/parameters/dyn_parameter.h>

using namespace Eigen;
namespace symbolic_functions
{
    Matrix<double,Nx,Nx> dfdx(Matrix<double,Nx,1> x_, Matrix<double,Nu,1>u)
    {
        float x = x_(0,0);
        float y = x_(1,0);
        float v = x_(2,0);
        float del = x_(3,0);
        float th = x_(4,0);

        float a = u(0,0);
        float deldot = u(1,0);

        Matrix<double,Nx,Nx> linA;
        linA.setZero();
        linA(0,2) = std::cos(th);
        linA(0,4) = -v * std::sin(th);
        linA(1,2) = std::sin(th);
        linA(1,4) = v * cos(th);
        linA(4,2) = std::tan(del)*invL;
        linA(4,3) = v*invL/(std::cos(del)*std::cos(del));

        return linA;
    }
}



#endif