#ifndef INITIAL_GUESS_H
#define INITIAL_GUESS_H
#include <qpOASES.hpp>
#include <iostream>
#include <chrono>
#include <Eigen/Dense>
#include <optimization_module/problem_description.h>
#include <optimization_module/utils/math.hpp>
#include <optimization_module/dimension.h>
#include <optimization_module/parameters/dyn_parameter.h>
#include <optimization_module/initial_guess.hpp>
#include <ros/ros.h>
// #include <optimization_module/symbolic_functions/dfdx.hpp>

template<const int Nx, const int Nu, const int N>
class QP
{
    protected:
    typedef Matrix<double,Nx,1> 	VectorX; 
    typedef Matrix<double,Nu,1> 	VectorU;
    typedef Matrix<double,Nx,Nx> 	MatrixXX;
    typedef Matrix<double,Nx,Nu> 	MatrixXU;
    typedef Matrix<double,Nu,Nx> 	MatrixUX;
    typedef Matrix<double,Nu,Nu> 	MatrixUU;

    public:
    QP(ProblemDescription<Nx, Nu>& prob,
       const VectorX x_init,
       const VectorU u_init, 
       const double Q, 
       const double R);
    void solve();
    Collection<VectorU,N> get_solution();

    VectorX                     x0_;                    // initial state
    Collection<Matrix<double,5,1>,N+1> local_wpts;
    // Collection<VectorX,N+1> 	xN_;					// nominal state
    // Collection<double,N+1> 		cN_; 					// nominal cost
    // Collection<VectorXd,N+1> 	constN_; 				// nominal constraint
    // Collection<VectorXd,N+1> 	activated_; 			// flag for active constraints
    VectorU              		u0_;					// initial input
    // Matrix<double,4,1> 			costsN_; 				// stage, final, penalty, lagrange
    Collection<VectorU,N> 		uN_;					// nominal input

    double Q_;
    double R_; 

    private:
    ProblemDescription<Nx,Nu>&  prob_; 	// dynamics, cost, constraints

};

template<const int Nx, const int Nu, const int N>
QP<Nx,Nu,N>::QP( ProblemDescription<Nx,Nu>& prob,						// problem setup
                const VectorX x_init,
                const VectorU u_init, const double Q, const double R) : prob_(prob), Q_(Q), R_(R)
{
    // initial setups
	x0_ = x_init;
	xN_[0] = x0_;
	u0_ = u_init;
	dt_ = dt;
	costsN_ = Matrix<double,4,1>::Zero();

}


template<const int Nx, const int Nu, const int N>
void QP<Nx,Nu,N>::solve( )
{   
    DynamicsDerivatives<Nx,Nu> dyn = prob_.dynamics(x0_, uN_, dt_);
    //dyn.fx -> A_fix (A*dt+I)
    //dyn.fu -> B_fix (B*dt)
    int_t total_N = (Nx+Nu)*(N+1);
    real_t err, tic, toc;
    real_t *x1 = new real_t[total_N];

    QP_block problem_guess = QP_block(local_wpts, Q_, R_);

    double* solution = problem_guess.QP_block_solver(x0_, u0_);
    
    for(int i=0; i< N; i++)
    {    
        double u1 = solution[6*(N+1)+2*i];
        double u2 = solution[6*(N+1)+2*i+1];   
        VectorU u_(u1, u2);
        uN_[i] = u_;
    }

}


template<const int Nx, const int Nu, const int N>
Collection<VectorU,N> QP<Nx,Nu,N>::get_solution( )
{
    return uN_;
}


#endif