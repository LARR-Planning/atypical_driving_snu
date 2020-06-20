#ifndef CONX_FINAL_HPP
#define CONX_FINAL_HPP

#include <Eigen/Dense>
#include <optimization_module/dimension.h>
#include <optimization_module/parameters/dyn_parameter.h>

using namespace Eigen;
namespace symbolic_functions
{
    Matrix<double,Nc-4,Nx> conx_final(Matrix<double,Nx,1> x_, Matrix<double,4,1>
            sfc_modified)
    {
        float x = x_(0,0);
        float y = x_(1,0);
        float v = x_(2,0);
        float a = x_(3,0);
        float del = x_(4,0);
        float th = x_(5,0);

        Matrix<double,Nc-4,Nx> CONX_FINAL;

        CONX_FINAL.setZero();
        CONX_FINAL(0,0) = 1.0;
        CONX_FINAL(1,0) = -1.0;
        CONX_FINAL(2,1) = 1.0; 
        CONX_FINAL(3,1) = -1.0;

        CONX_FINAL(4,3) = 1.0;
        CONX_FINAL(5,3) = -1.0;

        CONX_FINAL(6,4) = 1.0;
        CONX_FINAL(7,4) = -1.0;
        return CONX_FINAL;
    }
}




#endif
