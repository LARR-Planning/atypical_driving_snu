#ifndef CONX_FINAL_HPP
#define CONX_FINAL_HPP

#include <Eigen/Dense>
#include <optimization_module/dimension.h>
#include <optimization_module/parameters/dyn_parameter.h>

using namespace Eigen;
namespace symbolic_functions
{
    Matrix<double,Nc,Nx> conx_final(Matrix<double,Nx,1> x_,Matrix<double,Nu,1> u,Matrix<double, 4, 1> sfc_modified_temp)
    {
        float x = x_(0,0);
        float y = x_(1,0);
        float v = x_(2,0);
        float del = x_(3,0);
        float th = x_(4,0);

        float a = u(0,0);
        float deldot = u(1,0);
        Matrix<double,Nc,Nx> CONX_FINAL;

        CONX_FINAL.setZero();
        CONX_FINAL(0,3) = 1.0;
        CONX_FINAL(1,3) = -1.0;
        CONX_FINAL(4,0) = 1.0; // sfc xl
        CONX_FINAL(5,0) = -1.0; // sfc xu
        CONX_FINAL(6,1) = 1.0; // sfc yl
        CONX_FINAL(7,1) = -1.0; //sfc yu
        return CONX_FINAL;
    }
}




#endif