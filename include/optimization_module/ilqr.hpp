#ifndef ILQR_HPP
#define ILQR_HPP

#include <iostream>
#include <chrono>
#include <Eigen/Dense>
#include <optimization_module/problem_description.h>
#include <optimization_module/utils/math.hpp>
//#define EIGEN_USE_MKL_ALL

// using namespace Eigen;

struct iLQRParams
{
    double rho ;
    double drho;
    double rhoFactor;
    double rhoMax;
    double rhoMin;
    Eigen::VectorXd tolGrads;
    Eigen::VectorXd tolCosts;
    Eigen::VectorXd tolConsts;
    Eigen::VectorXd alphas;
    int maxIter;
    double mu;
    double lambda;
    double phi;
    int verbosity;
    double dmu;
    double dphi;
};


template<const int Nx, const int Nu, const int N>
class iLQR
{	
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	protected:
		typedef Eigen::Matrix<double,Nx,1> 	VectorX; 
		typedef Eigen::Matrix<double,Nu,1> 	VectorU;
		typedef Eigen::Matrix<double,Nx,Nx> 	MatrixXX;
		typedef Eigen::Matrix<double,Nx,Nu> 	MatrixXU;
		typedef Eigen::Matrix<double,Nu,Nx> 	MatrixUX;
		typedef Eigen::Matrix<double,Nu,Nu> 	MatrixUU;
		
		struct FullTrajectory
		{
			Collection<VectorX,N+1> x;
			Collection<double,N+1> c;
			Collection<Eigen::VectorXd,N+1> con;
			Collection<Eigen::VectorXd,N+1> activated;
			Collection<VectorU,N> u;
			Eigen::Matrix<double,4,1> costs;
		};

	public: 
		iLQR( ProblemDescription<Nx,Nu>&,
			  const VectorX, 
			  const Collection<VectorU,N>, 
			  const double,
			  const iLQRParams);
		void 						solve();				// main function
		
		double 						get_t_all();
		double						get_t_forward();
		double						get_t_backward();

		VectorX 					x0_; 					// initial state
		Collection<VectorX,N+1> 	xN_;					// nominal state
		Collection<double,N+1> 		cN_; 					// nominal cost
		Collection<Eigen::VectorXd,N+1> 	constN_; 				// nominal constraint
		Collection<Eigen::VectorXd,N+1> 	activated_; 			// flag for active constraints
		Collection<VectorU,N> 		uN_;					// nominal input
		Eigen::Matrix<double,4,1> 			costsN_; 				// stage, final, penalty, lagrange

		Collection<Eigen::VectorXd,N+1> 	mu_; 					// penalty
		Collection<Eigen::VectorXd,N+1> 	lambda_; 				// lagrange
		Collection<Eigen::VectorXd,N+1> 	phi_; 					// penalty/lagrange update condition
		Collection<Eigen::VectorXd,N+1> 	eq_flag_; 				// flag for equality constrints
		
		Collection<VectorU,N> 		k_;						// feedforward term
		Collection<MatrixUX,N> 		K_;						// feedback gain

	private:
		FullTrajectory 				forward_pass(double);	// forward pass
		Eigen::Matrix<double,2,1> 			backward_pass(); 		// backward pass
		void 						update_lagrange(int);	// lagrange update routine

		iLQRParams 					params_;				// structure of parameters
		double 						dt_;					// time step

		int 						max_outer_iter_;
		double 						rho_; 	
		double 						drho_;

		double 						t_forward_;
		double					 	t_backward_;
		double 						t_all;

		ProblemDescription<Nx,Nu>&  prob_; 	// dynamics, cost, constraints
};

// ADDED
template<const int Nx, const int Nu, const int N>
double iLQR<Nx,Nu,N>::get_t_all()
{
	return t_all;
}
template<const int Nx, const int Nu, const int N>
double iLQR<Nx,Nu,N>::get_t_forward()
{
	return t_forward_;
}
template<const int Nx, const int Nu, const int N>
double iLQR<Nx,Nu,N>::get_t_backward()
{
	return t_backward_;
}

template<const int Nx, const int Nu, const int N>
iLQR<Nx,Nu,N>::iLQR( ProblemDescription<Nx,Nu>& prob,						// problem setup
					 const VectorX x_init,  								// initial state
					 const Collection<VectorU,N> u_init, 					// initial input guess
					 const double dt, 										// time step
					 const iLQRParams params) 								// parameters
: prob_( prob )
{	
	// initial setups
	x0_ = x_init;
	xN_[0] = x0_;
	uN_ = u_init;
	dt_ = dt;
	costsN_ = Eigen::Matrix<double,4,1>::Zero();
	
	// update parameters
	params_ = params;
	max_outer_iter_ = params_.tolGrads.size();
	rho_ = params_.rho;
	drho_ = params_.drho;

	ConstraintDerivatives<Nx,Nu> const_deriv;
	// initial rollout
	for( int k=0; k<N+1; k++ )
	{
		if( k<N )
		{	// stage
			xN_[k+1] 	= ( prob_.dynamics(xN_[k], uN_[k], dt_) ).f;
			cN_[k] 		= ( prob_.cost(xN_[k], uN_[k], k) ).c;
			const_deriv = prob_.constraint(xN_[k], uN_[k], k);
			constN_[k] 	= const_deriv.con;
			
			costsN_(0) += cN_[k];
		}
		else
		{	// final
			cN_[k] 		= ( prob_.cost(xN_[k], nan(Nu,1), k) ).c;
			const_deriv = prob_.constraint(xN_[k], nan(Nu,1), k);
			constN_[k] 	= const_deriv.con;
		
			costsN_(1) = cN_[k];
		}
		mu_[k] 			= Eigen::VectorXd::Constant(const_deriv.Nc, params_.mu);
		lambda_[k] 		= Eigen::VectorXd::Constant(const_deriv.Nc, params_.lambda);
		phi_[k] 		= Eigen::VectorXd::Constant(const_deriv.Nc, params_.phi);
		eq_flag_[k] 	= Eigen::VectorXd::Constant(const_deriv.Nc, 0.0);
		activated_[k] 	= Eigen::VectorXd::Constant(const_deriv.Nc, 0.0);
		for( int i=0; i<const_deriv.Nc; i++ )
		{
			if( i < const_deriv.Neq )
				eq_flag_[k](i) = 1.0;
			if( i < const_deriv.Neq || lambda_[k](i) > 0.0 || constN_[k](i) < 0.0 )
				activated_[k](i) = 1.0;
		}

		costsN_(2) += 0.5 * (activated_[k].array() * mu_[k].array() * 
				constN_[k].array() * constN_[k].array()).sum();
		costsN_(3) += (lambda_[k].array() * constN_[k].array()).sum();
	}
}

template<const int Nx, const int Nu, const int N>
void iLQR<Nx,Nu,N>::solve( void )
{
	if( params_.verbosity > 0 )
		printf("%-6s%-12s%-12s%-12s%-12s%-12s%-12s%-12s%-12s%-12s%-12s\n", 
				"ITER", "COST", "STAGE", "FINAL", "PENALTY", "LAGRANGE",
				"REDUCTION", "EXPECTED", "GRADIENT", "LOG10(RHO)", "ALPHA");
	
	int iter = 1;
	int outer_iter = 1;
	auto t_start = std::chrono::system_clock::now();
	t_forward_ = 0.0;
	t_backward_ = 0.0;

	while(1) 	/* Outer process */
	{
		while(1) 	/* Inner process */
		{	
			/* STEP 1 : backward computation of optimal gains */
			auto t_backward_start = std::chrono::system_clock::now();
			Eigen::Matrix<double,2,1> dV = backward_pass();
			//cout << "Backward : " << 
			//	(std::chrono::system_clock::now() - t_backward_start).count()*1e-9 << endl;
			t_backward_ += (std::chrono::system_clock::now() - t_backward_start).count()*1e-9;

			/* STEP 2 : forward pass with linesearch */
			auto t_forward_start = std::chrono::system_clock::now();
			bool forward_done = false;
			double dcost = 0.0;
			double expected = 0.0;
			double alpha = 0.0;
			FullTrajectory new_traj;	
			for(int i=0; i<params_.alphas.size(); i++)
			{
				alpha = params_.alphas[i];
				new_traj = forward_pass( alpha );

				dcost = costsN_.sum() - new_traj.costs.sum();
				expected = -alpha*(dV(0) + alpha*dV(1));
				double ratio;
				if( expected > 0 )
					ratio = dcost / expected;
				else
					ratio = dcost / abs(dcost);

				if( ratio > 0 )
				{
					forward_done = true;
					break;
				}
			}	
			Eigen::Matrix<double,N,1> grad = Eigen::Matrix<double,N,1>::Zero(); // estimated gradient
			for(int k=0; k<N; k++)
				grad(k) = (alpha*k_[k].array().abs()/(uN_[k].array().abs()+1)).matrix().maxCoeff();
			double mean_grad = grad.mean();
			t_forward_ += (std::chrono::system_clock::now() - t_forward_start).count()*1e-9;
			
			/* STEP 3 : determine update trajectory or not */
			if( forward_done )
			{	// accept this step
				if( params_.verbosity > 0 )
					printf(
				"%-6d%-12.4g%-12.4g%-12.4g%-12.4g%-12.4g%-12.4g%-12.4g%-12.4g%-12.1f%-12.4g\n", 
							iter, costsN_.sum(), costsN_(0), costsN_(1), costsN_(2), costsN_(3),
							dcost, expected, mean_grad, std::log10(rho_), alpha);
				
				xN_ = new_traj.x;
				uN_ = new_traj.u;
				cN_ = new_traj.c;
				costsN_ = new_traj.costs;
				constN_ = new_traj.con;
				activated_ = new_traj.activated;

				// check termination
				if( mean_grad < params_.tolGrads(outer_iter-1) )
				{
					if( params_.verbosity > 0 )
						printf("[Inner] Converged (grad < tolGrad)\n");
					iter++;
					break;
				}
				if( std::abs(dcost) < params_.tolCosts(outer_iter-1) )
				{
					if( params_.verbosity > 0 )
						printf("[Inner] Converged (dcost < tolCost)\n");
					iter++;
					break;
				}

				// decrease rho
				drho_ = std::min( drho_ / params_.rhoFactor, 1.0 / params_.rhoFactor );
				rho_ = std::min( rho_ * drho_, params_.rhoMin);
				if( rho_ < params_.rhoMin )
					rho_ = params_.rhoMin;
			}
			else
			{	// ignore this step and increase rho
				drho_ = std::max( drho_ * params_.rhoFactor, params_.rhoFactor );
				rho_ *= drho_;
				if( rho_ > params_.rhoMax )
				{
					if( params_.verbosity > 0 )
						printf("[Inner] Exit (rho > rhoMax)\n");
					iter++;
					break;
				}

				if( params_.verbosity > 0 )
					printf("[Inner] Exit (minimum line search)\n");
				break;
			}

			// check maximum iteration during inner process
			if( iter == params_.maxIter )
			{
				if( params_.verbosity > 0 )
					printf("[Inner] Exit (maximum iteration)\n");
				break;
			}
			iter++;

		}// End Inner process

		outer_iter++;
		if( outer_iter > max_outer_iter_ )
		{
			outer_iter = max_outer_iter_;
			if( params_.verbosity > 0 )
				printf("[Outer] Finished\n");
			break;
		}
	
		// update penalty coefficients and lagrange multipliers
		update_lagrange( outer_iter );

	}// End Outer process	
	
	auto t_end = std::chrono::system_clock::now();
	// double t_all = (t_end - t_start).count()*1e-9;
	t_all = (t_end - t_start).count()*1e-9;

	if( params_.verbosity > 0 )
	{
		cout << "Total computation time : " << t_all << " seconds.\n";
		cout << "Forward pass : " << t_forward_ << " seconds. (" 
			<< (t_forward_/t_all)*100.0 << " %)\n";
		cout << "Backward pass : " << t_backward_ << " seconds. (" 
			<< (t_backward_/t_all)*100.0 << " %)\n";
		cout << "Etc : " << t_all - (t_forward_ + t_backward_) << " seconds. (" 
			<< (t_all - (t_forward_ + t_backward_))/t_all * 100.0 << " %)\n";
		cout << "Mean computation time for inner loop : " << t_all / outer_iter << " seconds.\n"; 
	}
}

/*
 * Forward Pass
 * generates xnew, unew, cnew, constnew from x0, uN, k, K, and alpha
 * returns FullTrajectory struct
 */
template<const int Nx, const int Nu, const int N>
typename iLQR<Nx,Nu,N>::FullTrajectory iLQR<Nx,Nu,N>::forward_pass( double alpha )
{
	FullTrajectory new_traj;
	
	// fix initial state
	new_traj.x[0] = x0_;
	new_traj.costs = Eigen::Matrix<double,4,1>::Zero();

	// integration
	for( int k=0; k<N+1; k++ )
	{
		if( k<N )
		{
			VectorX dx = new_traj.x[k] - xN_[k]; 
			VectorU du = alpha * k_[k] + K_[k] * dx; 
			new_traj.u[k] = uN_[k] + du;

			// new cost and constraints
			new_traj.c[k] 		= ( prob_.cost(new_traj.x[k], new_traj.u[k], k) ).c;
			new_traj.con[k] 	= ( prob_.constraint(new_traj.x[k], new_traj.u[k], k) ).con;
			new_traj.x[k+1] 	= ( prob_.dynamics(new_traj.x[k], new_traj.u[k], dt_) ).f;
			
			new_traj.costs(0) += new_traj.c[k];
		}
		else
		{
			new_traj.c[k] 		= ( prob_.cost(new_traj.x[k], nan(Nu,1), k) ).c;
			new_traj.con[k] 	= ( prob_.constraint(new_traj.x[k], nan(Nu,1), k) ).con;
			
			new_traj.costs(1) = new_traj.c[k];
		}
		
		new_traj.activated[k] = Eigen::VectorXd::Zero(new_traj.con[k].size());
		for( int i=0; i<new_traj.con[k].size(); i++ )
		{
			if( eq_flag_[k](i) == 1.0 || lambda_[k](i) > 0.0 || new_traj.con[k](i) < 0.0 )
				new_traj.activated[k](i) = 1.0;
		}
		
		new_traj.costs(2) += 0.5 * (new_traj.activated[k].array() * mu_[k].array() * 
				new_traj.con[k].array() * new_traj.con[k].array()).sum();
		new_traj.costs(3) += (lambda_[k].array() * new_traj.con[k].array()).sum();
	}
	
	return new_traj;
}

/* 
 * Backward Pass
 * computes optimal control inputs and saves in K_ and k_
 * returns estimated value reduction dV 
 */
template<const int Nx, const int Nu, const int N>
Eigen::Matrix<double,2,1> iLQR<Nx,Nu,N>::backward_pass()
{
	Eigen::Matrix<double,2,1> dV = Eigen::VectorXd::Zero(2);
	
	DynamicsDerivatives<Nx,Nu> 		dyn_deriv;
	CostDerivatives<Nx,Nu> 			cost_deriv;
	ConstraintDerivatives<Nx,Nu> 	const_deriv;

	// begin from final state
	cost_deriv = prob_.cost(xN_[N], nan(Nu,1), N, WITH_DERIVATIVES);
	VectorX 	Vx = cost_deriv.cx;
	MatrixXX	Vxx = cost_deriv.cxx;

	const_deriv = prob_.constraint(xN_[N], nan(Nu,1), N, WITH_DERIVATIVES);
	if( const_deriv.Nc > 0 )
	{	// update when constraints exist
		Eigen::MatrixXd I_mu = (activated_[N].array() * mu_[N].array()).matrix().asDiagonal();
		Vx 	+= const_deriv.conx.transpose() * (I_mu * const_deriv.con - lambda_[N]);
		Vxx += const_deriv.conx.transpose() * (I_mu * const_deriv.conx);
	}
	
	// compute value gradient backwardly
	for(int k=N-1; k>=0; k--)
	{
		dyn_deriv = prob_.dynamics(xN_[k], uN_[k], dt_, WITH_DERIVATIVES);
		cost_deriv = prob_.cost(xN_[k], uN_[k], k, WITH_DERIVATIVES);
		
		// dynamics and cost derivatives computed at forward pass
		MatrixXX 	fx = dyn_deriv.fx;
		MatrixXU 	fu = dyn_deriv.fu;
		VectorX 	cx = cost_deriv.cx;
		VectorU 	cu = cost_deriv.cu;
		MatrixXX 	cxx = cost_deriv.cxx;
		MatrixXU 	cxu = cost_deriv.cxu;
		MatrixUU 	cuu = cost_deriv.cuu;

		// derivaties of estimated cost-to-go
		VectorX 	Qx = cx + fx.transpose() * Vx;
		VectorU 	Qu = cu + fu.transpose() * Vx;
		MatrixXX 	Qxx = cxx + fx.transpose() * Vxx * fx;
		//MatrixXU 	Qxu = cxu + fx.transpose() * (Vxx + rho_*MatrixXX::Identity()) * fu;
		//MatrixUU 	Quu = cuu + fu.transpose() * (Vxx + rho_*MatrixXX::Identity()) * fu;
		MatrixXU 	Qxu = cxu + fx.transpose() * Vxx * fu;
		MatrixUU 	Quu = cuu + fu.transpose() * Vxx * fu;

		const_deriv = prob_.constraint(xN_[k], uN_[k], k, WITH_DERIVATIVES);
		if( const_deriv.Nc > 0 )
		{	// update when constraints exist
			Eigen::VectorXd con = const_deriv.con;
			Eigen::MatrixXd conx = const_deriv.conx;
			Eigen::MatrixXd conu = const_deriv.conu;

			Eigen::MatrixXd I_mu = (activated_[k].array() * mu_[k].array()).matrix().asDiagonal();
			Qx += conx.transpose() * (I_mu * con - lambda_[k]);
			Qu += conu.transpose() * (I_mu * con - lambda_[k]);
			Qxx += conx.transpose() * I_mu * conx;
			Qxu += conx.transpose() * I_mu * conu;
			Quu += conu.transpose() * I_mu * conu;
		}
		
		// optimal gains
		//k_[k] = -Quu.inverse() * Qu;
		//K_[k] = -Quu.inverse() * Qxu.transpose();
		k_[k] = -(Quu + rho_*MatrixUU::Identity()).inverse() * Qu;
		K_[k] = -(Quu + rho_*MatrixUU::Identity()).inverse() * Qxu.transpose();

		// update estimated cost-to-go reduction
		dV(0) += (k_[k].transpose() * Qu)(0,0);
		dV(1) += 0.5*(k_[k].transpose() * Quu * k_[k])(0,0);
		Vx = Qx + K_[k].transpose() * Quu * k_[k] + K_[k].transpose() * Qu + Qxu * k_[k];
		// Vx = Qx + 2*K_[k].transpose() * Quu * k_[k] + K_[k].transpose() * Qu + Qxu * k_[k];
		Vxx = Qxx + K_[k].transpose() * Quu * K_[k] + 
			K_[k].transpose() * Qxu.transpose() + Qxu * K_[k];
		Vxx = 0.5*(Vxx + Vxx.transpose());
	}
	return dV;
}

/* 
 * Update Lagrange
 * lambda update : tolConst < v <= phi 
 * mu update : phi < v
 */
template<const int Nx, const int Nu, const int N>
void iLQR<Nx,Nu,N>::update_lagrange( int outer_iter )
{
	// costs related with constraints will be re-evaluated
	costsN_(2) = 0.0;
	costsN_(3) = 0.0;

	for(int k=0; k<N+1; k++)
	{
		//const VectorXd con = constN_[k];
		for(int i=0; i<constN_[k].size(); i++)
		{
			double v = ((double)((constN_[k](i) < 0.0) || eq_flag_[k](i))) 
				* std::abs(constN_[k](i));

			if( (v > params_.tolConsts(outer_iter-1)) && (v <= phi_[k](i)) )
			{	// update lambda and and decrease phi
				lambda_[k](i) += -mu_[k](i) * constN_[k](i);
				phi_[k](i) *= params_.dphi;
			}
			else if( v > phi_[k](i) )
			{	// update mu
				mu_[k](i) *= params_.dmu;
			}
		}

		// after updating penalty and lagrange, constraint costs must be re-evaluated 
		costsN_(2) += 0.5 * (activated_[k].array() * mu_[k].array() * 
				constN_[k].array() * constN_[k].array()).sum();
		costsN_(3) += (lambda_[k].array() * constN_[k].array()).sum();
	}
}

#endif
