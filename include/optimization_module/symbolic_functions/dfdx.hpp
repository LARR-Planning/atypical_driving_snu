#ifndef DFDX_HPP
#define DFDX_HPP

#include <Eigen/Dense>
#include <optimization_module/dimension.h>
#include <optimization_module/parameters/dyn_parameter.h>

// using namespace Eigen;
namespace symbolic_functions
{
    Eigen::Matrix<double,Nx,Nx> dfdx(Eigen::Matrix<double,Nx,1> x_, Eigen::Matrix<double,Nu,1>u_)
    {
        float x = x_(0,0);
        float y = x_(1,0);
        float v = x_(2,0);
        float a = x_(3,0);
        float del = x_(4,0);
        float th = x_(5,0);

        float adot = u_(0,0);
        float deldot = u_(1,0);

        Eigen::Matrix<double,Nx,Nx> linA;
        linA.setZero();
        linA(0,2) = std::cos(th);
        linA(0,5) = -v * std::sin(th);
        linA(1,2) = std::sin(th);
        linA(1,5) = v * cos(th);
        linA(2,3) = 1.0;
        linA(5,2) = std::tan(del)*invL;
        linA(5,4) = v*invL/(std::cos(del)*std::cos(del));

        return linA;
    }
}



#endif
