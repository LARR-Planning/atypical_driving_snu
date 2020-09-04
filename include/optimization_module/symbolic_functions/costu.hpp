#ifndef COSTU_HPP
#define COSTU_HPP

#include <Eigen/Dense>
#include <optimization_module/dimension.h>
#include <optimization_module/parameters/dyn_parameter.h>

// using namespace Eigen;
namespace symbolic_functions
{
    Eigen::Matrix<double,Nu,1> costu(Eigen::Matrix<double,Nx,1>x_,
                Eigen::Matrix<double,Nu,1>u_,
                Eigen::Matrix<double,Nx,1>Q,
                Eigen::Matrix<double,Nu,1>R,
                Eigen::Matrix<double,5,1>g
                )
    {
        //float x = x_(0,0);
        //float y = x_(1,0);
        //float v = x_(2,0);
        //float a = x_(3,0);
        //float del = x_(4,0);
        //float th = x_(5,0);
        
        float adot = u_(0,0);
        float deldot = u_(1,0);
        // float Q1 = Q(0);
        // float Q2 = Q(1);
        float R0 = R(0);
        float R1 = R(1);
        
        Eigen::Matrix<double,Nu,1> A0;
        A0(0,0) = R0*adot;
        A0(1,0) = R1*deldot;
        return A0;
    }
}


#endif
