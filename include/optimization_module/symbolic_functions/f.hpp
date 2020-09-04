#ifndef F_HPP
#define F_HPP

#include <Eigen/Dense>
#include <optimization_module/dimension.h>
#include <optimization_module/parameters/dyn_parameter.h>

// using namespace Eigen;
namespace symbolic_functions
{
    Eigen::Matrix<double,Nx,1> f(Eigen::Matrix<double,Nx,1>x_, Eigen::Matrix<double,Nu,1>u_)
    {
        float x = x_(0,0);
        float y = x_(1,0);
        float v = x_(2,0);
        float a = x_(3,0);
        float del = x_(4,0);
        float th = x_(5,0);
        
        float adot = u_(0,0);
        float deldot = u_(1,0);
        Eigen::Matrix<double,Nx,1> dyn;
        dyn.setZero();

        dyn(0,0) = v * std::cos(th);
        dyn(1,0) = v * std::sin(th);
        dyn(2,0) = a;
        dyn(3,0) = adot;
        dyn(4,0) = deldot;
        dyn(5,0) = v * std::tan(del)*invL; 
        
        return dyn;

    }
}

#endif
