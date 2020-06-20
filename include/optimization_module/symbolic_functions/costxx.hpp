#ifndef COSTXX_HPP
#define COSTXX_HPP


#include <Eigen/Dense>
#include <optimization_module/dimension.h>
#include <optimization_module/parameters/dyn_parameter.h>

using namespace Eigen;
namespace symbolic_functions
{
    Matrix<double,Nx,Nx> costxx(Matrix<double,Nx,1>x_,
                Matrix<double,Nu,1>u_,
                Matrix<double,Nx,1>Q,
                Matrix<double,Nu,1>R,
                Matrix<double,3,1>g
                )
    {
        float x = x_(0,0);
        float y = x_(1,0);
        float v = x_(2,0);
        float a = x_(3,0);
        float del = x_(4,0);
        float th = x_(5,0);
        
        // float a = u_(0,0);
        // float deldot = u_(1,0);
        
        float Q0 = Q(0);
        float Q1 = Q(1);
        float Q2 = Q(2);
        float Q3 = Q(3);
        float Q4 = Q(4);
        float Q5 = Q(5);

        // float R1 = R(0);
        // float R2 = R(1);

        //float dist = std::sqrt((x-obs(0))*(x-obs(0))+(y-obs(1))*(y-obs(1)));
        Matrix<double,Nx,Nx> A0;
        A0.setZero();
        A0(0,0) = Q0;
        A0(1,1) = Q1;
        A0(4,4) = Q4;
        A0(5,5) = Q5;
        return A0;
    }
}
#endif
