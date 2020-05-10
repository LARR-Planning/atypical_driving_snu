#ifndef SYSTEM_BASE_ATYPICAL_H
#define SYSTEM_BASE_ATYPICAL_H

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

/*
 * Base class of nonlinear system.
 * Specific description will be implemented later
 */

template<const int Nx, const int Nu>
class SystemBase
{
    protected:
        typedef Matrix<double,Nx,1> VectorX;
        typedef Matrix<double,Nu,1> VectorU;
        typedef Matrix<double,Nx,Nx> MatrixXX;
        typedef Matrix<double,Nx,Nu> MatrixXU;

    public:
        explicit SystemBase(){};
        ~SystemBase(){};

        virtual VectorX f(VectorX x_nominal = VectorX::Zero(),
                          VectorU u_nominal = VectorU::Zero())
        {   return VectorX::Zero();}
        
        virtual MatrixXX dfdx(VectorX x_nominal = VectorX::Zero(), 
						   VectorU u_nominal = VectorU::Zero()) 
		{	return MatrixXX::Identity(); }
		
		virtual MatrixXU dfdu(VectorX x_nominal = VectorX::Zero(), 
						   VectorU u_nominal = VectorU::Zero())
		{	return MatrixXU::Zero(); }

};
#endif
