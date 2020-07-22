#ifndef COSTUU_HPP
#define COSTUU_HPP

#include <Eigen/Dense>
#include <optimization_module/dimension.h>
#include <optimization_module/parameters/dyn_parameter.h>

using namespace Eigen;

namespace symbolic_functions
{
    Matrix<double,Nu,Nu> costuu(Matrix<double,Nx,1>x_,
                Matrix<double,Nu,1>u_,
                Matrix<double,Nx,1>Q,
                Matrix<double,Nu,1>R,
                Matrix<double,5,1>g
                )
    {
        float R1 = R(0);
        float R2 = R(1);

        Matrix<double,Nu,Nu> A0;
        A0.setZero();

        A0(0,0) = R1;
        A0(1,1) = R2;
        return A0;
    }
}


#endif