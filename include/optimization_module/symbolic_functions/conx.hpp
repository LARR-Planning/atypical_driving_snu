#ifndef CONX_HPP
#define CONX_HPP

#include <Eigen/Dense>
#include <optimization_module/dimension.h>
#include <optimization_module/parameters/dyn_parameter.h>

using namespace Eigen;
namespace symbolic_functions
{
    Matrix<double,Nc,Nx> conx(Matrix<double,Nx,1> x_,Matrix<double,Nu,1> u,
            Matrix<double,4,1> sfc_modified)
    {
        float x = x_(0,0);
        float y = x_(1,0);
        float v = x_(2,0);
        float a = x_(3,0);
        float del = x_(4,0);
        float th = x_(5,0);

        float adot = u(0,0);
        float deldot = u(1,0);

        Matrix<double,Nc,Nx> CONX;
        CONX.setZero();
        CONX(0,0) = 1.0; 
        CONX(1,0) = -1.0; 
        CONX(2,1) = 1.0;
        CONX(3,1) = -1.0;
        CONX(4,3) = 1.0; 
        CONX(5,3) = -1.0;
        CONX(6,4) = 1.0; 
        CONX(7,4) = -1.0;

        return CONX;
    }
}




#endif
