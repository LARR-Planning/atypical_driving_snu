#ifndef COST_HPP
#define COST_HPP

#include <Eigen/Dense>
#include <optimization_module/dimension.h>
#include <optimization_module/parameters/dyn_parameter.h>

using namespace Eigen;
namespace symbolic_functions
{
    double cost(Matrix<double,Nx,1>x_,
                Matrix<double,Nu,1>u_,
                Matrix<double,Nx,1>Q,
                Matrix<double,Nu,1>R,
                Matrix<double,5,1>g
                )
    {
        float x = x_(0,0);
        float y = x_(1,0);
        float v = x_(2,0);
        float a = x_(3,0);
        float del = x_(4,0);
        float th = x_(5,0);

        float adot = u_(0,0);
        float deldot = u_(1,0);
        
        float Q0 = Q(0); // x
        float Q1 = Q(1); // y
        float Q2 = Q(2); // v
        float Q3 = Q(3); // a
        float Q4 = Q(4); // steer
        float Q5 = Q(5); // theta
        
        float R0 = R(0);
        float R1 = R(1);

        //float dist = std::sqrt((x-obs(0))*(x-obs(0))+(y-obs(1))*(y-obs(1)));
        double cost_value = 0.5*Q0*(x-g(0))*(x-g(0)) + 0.5*Q1*(y-g(1))*(y-g(1))+0.5*Q3*(a-g(3))*(a-g(3))
                +0.5*Q4*(del-g(4))*(del-g(4))+0.5*Q5*(th-g(2))*(th-g(2)) + 0.5*R0*adot*adot + 0.5*R1*deldot*deldot;
        return cost_value;
    }
}
#endif
