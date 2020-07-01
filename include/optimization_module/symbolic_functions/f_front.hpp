#ifndef F_FRONT_HPP
#define F_FRONT_HPP

#include <Eigen/Dense>
#include <optimization_module/dimension.h>
#include <optimization_module/parameters/dyn_parameter.h>

using namespace Eigen;
namespace symbolic_functions
{
    Matrix<double,Nx,1> f_front(Matrix<double,Nx,1>x_, Matrix<double,Nu,1>u)
    {
        float x = x_(0,0);
        float y = x_(1,0);
        float v = x_(2,0);
        float a = x_(3,0);
        float del = x_(4,0);
        float th = x_(5,0);
        float adot = u(0,0);
        float deldot = u(1,0);
        Matrix<double,Nx,1> dyn;
        dyn.setZero();

        dyn(0,0) = v * std::cos(th)*cos(del);
        dyn(1,0) = v * std::sin(th)*cos(del);
        dyn(2,0) = a;
        dyn(3,0) = adot;
        dyn(4,0) = deldot;
        dyn(5,0) = v * std::sin(del)*invL;
        
        return dyn;

    }
}

#endif