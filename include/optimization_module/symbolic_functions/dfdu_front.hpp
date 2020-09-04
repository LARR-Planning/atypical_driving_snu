#ifndef DFDU_FRONT_HPP
#define DFDU_FRONT_HPP

#include <Eigen/Dense>
#include <optimization_module/dimension.h>
#include <optimization_module/parameters/dyn_parameter.h>

// using namespace Eigen;
namespace symbolic_functions
{
    Eigen::Matrix<double,Nx,Nu> dfdu_front(Eigen::Matrix<double,Nx,1>x_, Eigen::Matrix<double,Nu,1> u)
    {
        float x = x_(0,0);
        float y = x_(1,0);
        float v = x_(2,0);
        float a = x_(3,0);
        float del = x_(4,0);
        float th = x_(5,0);
        float adot = u(0,0);
        float deldot = u(1,0);

        Eigen::Matrix<double,Nx,Nu> linB;
        linB.setZero();
        linB(3,0) = 1.0;
        linB(4,1) = 1.0;

        return linB;
    }
}

#endif