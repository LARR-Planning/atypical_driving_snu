#ifndef UTILS_MATH_HPP
#define UTILS_MATH_HPP
#pragma once
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/MatrixFunctions>

#include <geometry_msgs/Quaternion.h> // I want to remove this....

using namespace Eigen;

// fixed size homogenous container
template<typename T, const int size>
using Collection = std::array<T, size>; 


VectorXd power(const double a, const VectorXd b);

//VectorXd power(const VectorXd a, const VectorXd b)
//{
//	int Na = a.size();
//	int Nb = b.size();
//	if( Na != Nb )
//		return VectorXd::Constant(1,0./0.);
//	else
//	{
//		VectorXd apowb(Na);
//		for(int i=0; i<Na; i++)
//			apowb(i) = std::pow(a(i), b(i));
//		return apowb;
//	}
//}

// TODO : can I make fixed-size version of this?
//VectorXd cat(const VectorXd& a, const VectorXd& b)
//{
//	VectorXd c;
//	c.resize( a.size() + b.size() );
//	c.block(0,0,a.size(),1) << a;
//	c.block(a.size(),0,b.size(),1) << b;
//	return c;
//}

// TODO : can I make fixed-size version of this?

MatrixXd nan(const int n, const int m);
/*
 *	Right kronecker product with identity matrix (I x A)
 */
//template<const int rowA, const int colA, const int N>
//Matrix<double,N*rowA,N*colA> right_kronecker( const Matrix<double,rowA,colA> A )
//{
//	Matrix<double,N*rowA,N*colA> kronA = Matrix<double,N*rowA,N*colA>::Zero();
//	for(int i=0; i<N; i++)
//		kronA.template block<rowA,colA>(i*rowA,i*colA) = A;
//	return kronA;
//}

//MatrixXd right_kronecker( const MatrixXd A )
//{
//	int N = A.rows();
//	MatrixXd kronA(N*N,N*N);
//	for(int i=0; i<N; i++)
//		kronA.block(i*N,i*N,N,N) = A;
//	return kronA;
//}

/*
 *	Left kronecker product with identity matrix (A x I)
 */
//template<const int rowA, const int colA, const int N>
//Matrix<double,N*rowA,N*colA> left_kronecker( const Matrix<double,rowA,colA> A )
//{
//	Matrix<double,N*rowA,N*colA> kronA = Matrix<double,N*rowA,N*colA>::Zero();
//	for(int i=0; i<rowA; i++)
//	{
//		for(int j=0; j<colA; j++)
//			kronA.template block<N,N>(i*N,j*N) = Matrix<double,N,1>::Constant(A(i,j)).asDiagonal();
//	}
//	return kronA;
//}

//MatrixXd left_kronecker( const MatrixXd A )
//{
//	int N = A.rows();
//	MatrixXd kronA(N*N,N*N);
//	for(int i=0; i<N; i++)
//	{
//		for(int j=0; j<N; j++)
//			kronA.block(i*N,j*N,N,N) = MatrixXd::Constant(N,1,A(i,j)).asDiagonal();
//	}
//	return kronA;
//}

/*
 *	Square root of positive symmetric matrix
 */
template<const int N>
Matrix<double,N,N> sqrt( const Matrix<double,N,N> Q )
{
	SelfAdjointEigenSolver<Matrix<double,N,N>> es(Q);
	return es.operatorSqrt();
}

/*
 *	Vectorization (column major)
 */
template<const int rowA, const int colA>
Matrix<double,rowA*colA,1> vec( Matrix<double,rowA,colA> A )
{
	Map<Matrix<double,rowA*colA,1>> vecA(A.data(),rowA,colA);
	return vecA;
}

/*
 *	Factorial (simple)
 */

int factorial(int n);
/*
 *	Matrix power (simple)
 */
template<const int N>
Matrix<double,N,N> pow( const Matrix<double,N,N> A, int p )
{
	if( p == 0 )
		return Matrix<double,N,N>::Identity();
	else
		return pow(A,p-1) * A;
}

/*
 *	expm_jac : Jacobian of matrix exponential
 */
template<const int N, const int size>
Collection<Matrix<double,N,N>,size>
	expm_jac( Matrix<double,N,N> A, 
			  Collection<Matrix<double,N,N>,size> dA_dK,
			  double dt )
{
	Collection<Matrix<double,N,N>,size> dexpAdt_dK;
	for(int k=0; k<size; k++)
	{
		dexpAdt_dK[k] = dt*dA_dK[k];
		for(int i=1; i<=2; i++) // this number must be different wit respect to dt
		{
			Matrix<double,N,N> deriv = Matrix<double,N,N>::Zero();
			for(int j=0; j<=i; j++)
				deriv += pow<N>(dt*A,j) * (dt*dA_dK[k]) * pow<N>(dt*A,(i-j));
			dexpAdt_dK[k] += (1.0/factorial(i+1))*deriv;
		}
	}
	return dexpAdt_dK;
}

//Matrix<double,3,1> q2e(const geometry_msgs::Quaternion q)
//{
//	Matrix<double,3,1> euler = Matrix<double,3,1>::Zero();
//	double ysqr = q.y * q.y;
//
//	double t0 = 2.0 * (q.w * q.x + q.y * q.z);
//	double t1 = 1.0 - 2.0 * (q.x * q.x + ysqr);
//	euler(0) = std::atan2(t0, t1);
//
//	double t2 = 2.0 * (q.w * q.y - q.z * q.x);
//	t2 = ((t2 > 1.0) ? 1.0 : t2);
//	t2 = ((t2 < -1.0) ? -1.0 : t2);
//	euler(1) = std::asin(t2);
//
//	double t3 = 2.0 * (q.w * q.z + q.x * q.y);
//	double t4 = 1.0 - 2.0 * (ysqr + q.z * q.z);
//	euler(2) = std::atan2(t3, t4);
//
//	return euler;
//}

//void e2q( const Matrix<double,3,1>& euler, Quaternion<double>& q )
//{
//	double roll = euler(0);
//	double pitch = euler(1);
//	double yaw = euler(2);
//
//	double cy = std::cos(yaw * 0.5);
//	double sy = std::sin(yaw * 0.5);
//	double cp = std::cos(pitch * 0.5);
//	double sp = std::sin(pitch * 0.5);
//	double cr = std::cos(roll * 0.5);
//	double sr = std::sin(roll * 0.5);
//
//	q.w() = cy*cp*cr + sy*sp*sr;
//	q.x() = cy*cp*sr - sy*sp*cr;
//	q.y() = sy*cp*sr + cy*sp*cr;
//	q.z() = sy*cp*cr - cy*sp*sr;
//}

//void e2q( const Matrix<double,3,1>& euler, geometry_msgs::Quaternion& q )
//{
//	double roll = euler(0);
//	double pitch = euler(1);
//	double yaw = euler(2);
//
//	double cy = std::cos(yaw * 0.5);
//	double sy = std::sin(yaw * 0.5);
//	double cp = std::cos(pitch * 0.5);
//	double sp = std::sin(pitch * 0.5);
//	double cr = std::cos(roll * 0.5);
//	double sr = std::sin(roll * 0.5);
//
//	q.w = cy*cp*cr + sy*sp*sr;
//	q.x = cy*cp*sr - sy*sp*cr;
//	q.y = sy*cp*sr + cy*sp*cr;
//	q.z = sy*cp*cr - cy*sp*sr;
//}

#endif
