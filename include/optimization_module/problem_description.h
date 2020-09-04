#ifndef PROBLEM_DESCRIPTION_H
#define PROBLEM_DESCRIPTION_H

#include <Eigen/Dense>

// using namespace Eigen;
using namespace std;

enum ReturnType { WITH_DERIVATIVES, WITHOUT_DERIVATIVES };

// base class for type definition
template<const int Nx, const int Nu>
class DescriptionBase 
{
	protected:
		typedef Eigen::Matrix<double,Nx,1> VectorX;
		typedef Eigen::Matrix<double,Nu,1> VectorU;
		typedef Eigen::Matrix<double,Nx,Nx> MatrixXX;
		typedef Eigen::Matrix<double,Nx,Nu> MatrixXU;
		typedef Eigen::Matrix<double,Nu,Nx> MatrixUX;
		typedef Eigen::Matrix<double,Nu,Nu> MatrixUU;
};

// dynamics container
template<const int Nx, const int Nu>
class DynamicsDerivatives : public DescriptionBase<Nx,Nu>
{
	public:
		using typename DescriptionBase<Nx,Nu>::VectorX;
		using typename DescriptionBase<Nx,Nu>::VectorU;
		using typename DescriptionBase<Nx,Nu>::MatrixXX;
		using typename DescriptionBase<Nx,Nu>::MatrixXU;
		using typename DescriptionBase<Nx,Nu>::MatrixUU;
		
		DynamicsDerivatives(){};
		VectorX 	f = VectorX::Zero();
		MatrixXX 	fx = MatrixXX::Zero();
		MatrixXU	fu = MatrixXU::Zero();
};

// cost container
template<const int Nx, const int Nu>
class CostDerivatives : public DescriptionBase<Nx,Nu>
{
	public:
		using typename DescriptionBase<Nx,Nu>::VectorX;
		using typename DescriptionBase<Nx,Nu>::VectorU;
		using typename DescriptionBase<Nx,Nu>::MatrixXX;
		using typename DescriptionBase<Nx,Nu>::MatrixXU;
		using typename DescriptionBase<Nx,Nu>::MatrixUU;

		CostDerivatives(){};
		double 		c = 0;
		VectorX 	cx = VectorX::Zero();
		VectorU		cu = VectorU::Zero();
		MatrixXX 	cxx = MatrixXX::Zero();
		MatrixXU 	cxu = MatrixXU::Zero();
		MatrixUU 	cuu = MatrixUU::Zero();
};

// constraint container (row size is determined at runtime)
template<const int Nx, const int Nu>
class ConstraintDerivatives : public DescriptionBase<Nx,Nu>
{
	public:
		using typename DescriptionBase<Nx,Nu>::VectorX;
		using typename DescriptionBase<Nx,Nu>::VectorU;
		using typename DescriptionBase<Nx,Nu>::MatrixXX;
		using typename DescriptionBase<Nx,Nu>::MatrixXU;
		using typename DescriptionBase<Nx,Nu>::MatrixUU;

		ConstraintDerivatives(){ ConstraintDerivatives(0,0); }
		ConstraintDerivatives(const int nc, const int neq)
		{	// the number of all and equality constraints are determined at construction
			Nc = nc;
			Neq = neq;
			con = Eigen::MatrixXd::Zero(nc,1);
			conx = Eigen::MatrixXd::Zero(nc,Nx);
			conu = Eigen::MatrixXd::Zero(nc,Nu);
		};
		int Nc;
		int Neq;
		Eigen::Matrix<double,Eigen::Dynamic,1> 	con;
		Eigen::Matrix<double,Eigen::Dynamic,Nx> 	conx;
		Eigen::Matrix<double,Eigen::Dynamic,Nu> 	conu;
};

// base class for describing dynamics, cost, and constraint
template<const int Nx, const int Nu>
class ProblemDescription : public DescriptionBase<Nx,Nu>
{
	public:
		using typename DescriptionBase<Nx,Nu>::VectorX;
		using typename DescriptionBase<Nx,Nu>::VectorU;
		using typename DescriptionBase<Nx,Nu>::MatrixXX;
		using typename DescriptionBase<Nx,Nu>::MatrixXU;
		using typename DescriptionBase<Nx,Nu>::MatrixUU;

		explicit ProblemDescription(){};
		~ProblemDescription(){};

		/*
		 * <dynamics, cost, constraint function>
		 * Those functions return either value or derivative according to ReturnType argument.
		 * This is because preventing unnecessary computation of derivatives, 
		 * when those functions are called during forward pass of iLQR
		 */
		virtual DynamicsDerivatives<Nx,Nu> 
			dynamics(const VectorX x, 
					 const VectorU u, 
					 const double dt, 
					 const ReturnType type = WITHOUT_DERIVATIVES){}

		virtual CostDerivatives<Nx,Nu> 
			cost(const VectorX x, 
				 const VectorU u,
				 const int idx = 0,
				 const ReturnType type = WITHOUT_DERIVATIVES){}

		virtual ConstraintDerivatives<Nx,Nu>
			constraint(const VectorX x, 
					   const VectorU u,
					   const int idx = 0,
					   const ReturnType type = WITHOUT_DERIVATIVES)
		{ 	// basically return zero constraints
			return ConstraintDerivatives<Nx,Nu>(0,0);
		}
};

#endif
