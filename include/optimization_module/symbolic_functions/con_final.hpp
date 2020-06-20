#ifndef CON_FINAL_HPP
#define CON_FINAL_HPP

#include <Eigen/Dense>
#include <optimization_module/dimension.h>
#include <optimization_module/parameters/dyn_parameter.h>

using namespace Eigen;
namespace symbolic_functions
{
    Matrix<double,Nc-4,1> con_final(Matrix<double,Nx,1> x_, Matrix<double,4,1>
            sfc_modified)
    {
        float x = x_(0,0);
        float y = x_(1,0);
        float v = x_(2,0);
        float a = x_(3,0);
        float del = x_(4,0);
        float th = x_(5,0);

        Matrix<double,Nc-4,1> CON_FINAL;
        CON_FINAL.setZero();
        
        CON_FINAL(0,0) = x-(sfc_modified(0,0)-dist_safe);
        CON_FINAL(1,0) = (sfc_modified(1,0)+dist_safe)-x;
        CON_FINAL(2,0) = y-(sfc_modified(2,0)-dist_safe);
        CON_FINAL(3,0) = (sfc_modified(3,0)+dist_safe)-y;
        CON_FINAL(4,0) = a - acc_min;
        CON_FINAL(5,0) = acc_max - a;
        CON_FINAL(6,0) = del-steer_min;
        CON_FINAL(7,0) = steer_max - del;

        return CON_FINAL;
    }
}
#endif
