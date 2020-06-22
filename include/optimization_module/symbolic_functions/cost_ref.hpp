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
                Matrix<double,3,1>g
                )
    {
        float x = x_(0,0);
        float y = x_(1,0);
        float v = x_(2,0);
        float del = x_(3,0);
        float th = x_(4,0);
        float a = u_(0,0);
        float deldot = u_(1,0);
        float Q1 = Q(0);
        float Q2 = Q(1);
        float Q3 = Q(3);
        float R1 = R(0);
        float R2 = R(1);

        //float dist = std::sqrt((x-obs(0))*(x-obs(0))+(y-obs(1))*(y-obs(1)));
        double cost_value = 0.5*Q1*(x-g(0))*(x-g(0))+0.5*Q2*(y-g(1))*(y-g(1))+0.5*Q3*del*del+0.5*R1*a*a + 0.5 * R2 * deldot*deldot;
        return cost_value;
    }
}
#endif