#ifndef BASIC_OPS
#define BASIC_OPS


#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <limits>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <sstream>


#include <Eigen/Core>
#include <Eigen/Geometry>


#include <unsupported/Eigen/Splines>


/**
 * Created by JBS In 2020/1/16
 * icsl-Jeon/github.com
 * 
 * @brief This code includes necessary operation modules for Detection-Aware Planning (DAP)
 * 
 */
using namespace std;

string current_working_directory();

namespace DAP{

    typedef Eigen::Matrix<float,3,3> CameraMatrix; 
    typedef Eigen::Transform<float,3,Eigen::Affine> TransformMatrix;
    typedef Eigen::Matrix<float,8,-1> TXYZQuatTraj; // t,x,y,z,qw,qx,qy,qz
    typedef Eigen::Matrix<float,4,-1> TXYZTraj; // t,x,y,z
    typedef Eigen::Spline<float,8> Spline8f; 

    struct Index{
        int r;
        int c;
        Index(uint r, uint c): r(r),c(c) {};
        bool operator==(const Index & rhs) {return (rhs.r == r and rhs.c ==c);};
    };

     /**
     * @brief find non-zero pixel in bgr 3 channel image  
     * 
     * @param input_array 
     * @param output_array (N_nnz,2) matrix
     */
    void mean_var_update(int * count , float *mean ,float * var,float new_val);
    vector<TransformMatrix> read_SE3_seq(string file_name);

    TXYZQuatTraj read_TXYZQuat_seq(string file_name);

    Eigen::VectorXf polyfit(Eigen::VectorXf xvals,Eigen::VectorXf yvals,int order);
    float polyeval(Eigen::VectorXf coeffs, float x);
    float polyeval_derivative(Eigen::VectorXf coeffs, float x);
}

#endif