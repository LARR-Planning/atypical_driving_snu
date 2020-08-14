#ifndef INITIAL_GUESS_H
#define INITIAL_GUESS_H
#include <qpOASES.hpp>
#include <iostream>
#include <chrono>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <optimization_module/problem_description.h>
#include <optimization_module/utils/math.hpp>
#include <optimization_module/dimension.h>
#include <optimization_module/parameters/dyn_parameter.h>
#include <ros/ros.h>
// #include <optimization_module/symbolic_functions/dfdx.hpp>


template<const int Nx, const int Nu, const int N>
class QP
{
    protected:
    typedef Triplet<double>  Trip;
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
        Collection<Matrix<double,5,1>,N+1> local_wpts_, 
        Vector2d qp_param);
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
    double dt_;

    private:
    ProblemDescription<Nx,Nu>&  prob_; 	// dynamics, cost, constraints

};

template<const int Nx, const int Nu, const int N>
QP<Nx,Nu,N>::QP( ProblemDescription<Nx,Nu>& prob,						// problem setup
                const VectorX x_init,
                const VectorU u_init, 
                Collection<Matrix<double,5,1>,N+1> local_wpts_, 
                Vector2d qp_param)
                 : prob_(prob), local_wpts(local_wpts_)
{
    // initial setups
	x0_ = x_init;
	// xN_[0] = x0_;
	u0_ = u_init;
	dt_ = dt;
    Q_ = qp_param(0,0); R_ = qp_param(1,0);

}


template<const int Nx, const int Nu, const int N>
void QP<Nx,Nu,N>::solve( )
{   
    Eigen::Matrix<double, Nx*(N+1),1> ref;
    for(int i=0; i<N+1; i++)
    {
        ref(6*i,0) = local_wpts[i][0];
        ref(6*i+1,0) = local_wpts[i][1]; 
        ref(6*i+2,0) = local_wpts[i][2];
        ref(6*i+3,0) = local_wpts[i][3];
        ref(6*i+4,0) = local_wpts[i][4];
        ref(6*i+5,0) = 0.0; 
    }

    DynamicsDerivatives<Nx,Nu> dyn = prob_.dynamics(x0_, u0_, dt_);
    //dyn.fx -> A_fix (A*dt+I)
    //dyn.fu -> B_fix (B*dt)

    Eigen::Matrix<double, Nx, Nx, RowMajor> A = symbolic_functions::dfdx(x0_, u0_);
    Eigen::Matrix<double, Nx, Nu, RowMajor> B = symbolic_functions::dfdu(x0_, u0_);
    Eigen::Matrix<double, Nx, 1> f0 = symbolic_functions::f(x0_, u0_);
    // Eigen::Matrix<double, Nx, Nx, RowMajor> A_fix = A*dt + Eigen::Matrix<double, Nx, Nx, RowMajor>::Identity();
    Eigen::Matrix<double, Nx, Nx, RowMajor> A_fix = dyn.fx;
    // Eigen::Matrix<double, Nx, Nu, RowMajor> B_fix = B*dt; 
    Eigen::Matrix<double, Nx, Nu, RowMajor> B_fix = dyn.fu;

    Eigen::Matrix<double, Nx, 1> f_fix = -(A*x0_*dt +B_fix*u0_) + f0 *dt;

    std::vector<Trip> trpQ; // Cost Matrix 
    std::vector<Trip> trpR; // Cost Matrix 
    
    std::vector<Trip> trpA; // A_eq
    std::vector<Trip> trpB; // B_eq 
    Trip tmp_A1;  Trip tmp_A2; Trip tmp_A3; Trip tmp_B1; Trip trmp_Q; Trip trmp_R;

    //Cost Matrix Generation 
    double Q = Q_; double R = R_; 
    for(int i=0; i<N+1; ++i){
        for(int j=0; j<2; ++j){
            trmp_Q = Trip(Nx*i+j, Nx*i+j, Q);
            trpQ.push_back(trmp_Q);
            trmp_R = Trip(Nu*i+j, Nu*i+j, R);
            trpR.push_back(trmp_R);
        }
    }
    Eigen::SparseMatrix<double, RowMajor> Q_sparse( (Nx)*(N+1), (Nx)*(N+1) );
    Q_sparse.setFromTriplets(trpQ.begin(), trpQ.end());

    Eigen::Matrix<double, -1, -1, RowMajor> Q_dense = MatrixXd(Q_sparse);

    Eigen::SparseMatrix<double, RowMajor> R_sparse(Nu*(N+1), Nu*(N+1));
    R_sparse.setFromTriplets(trpR.begin(), trpR.end());

    Eigen::Matrix<double, -1, -1, RowMajor> A_big = Eigen::Matrix<double, Nx*(N+1), Nx*(N+1),RowMajor>::Zero();
    Eigen::Matrix<double, -1, -1, RowMajor> B_big = Eigen::Matrix<double, Nx*(N+1), Nu*(N+1),RowMajor>::Zero();
    
    for(int i=0; i<N+1; i++)
    {
        Eigen::Matrix<double, Nx, Nx,RowMajor> A_tmp = A_fix.inverse();
        for(int j=i-1; j>=0; j--){
            A_tmp = A_tmp * A_fix;
            A_big.block(Nx*i, Nx*j, Nx, Nx) = A_tmp;
        }

        B_big.block(Nx*i, Nu*i, Nx, Nu) = B_fix; 
    }

    Eigen::Matrix<double, Nu*(N+1), Nu*(N+1), RowMajor> H_cost;
    H_cost = B_big.transpose()*A_big.transpose()*Q_sparse*A_big*B_big + R_sparse;
    H_cost = 1.0/2.0*(H_cost + H_cost.transpose());

    Eigen::Matrix<double, Nx*(N+1), 1> X_refined; 
    Eigen::Matrix<double, Nx, 1> Ax_Bu = A_fix*x0_ + B_fix*u0_;
    Eigen::Matrix<double, Nx, Nx,RowMajor> A_tmp = Eigen::Matrix<double, Nx, Nx, RowMajor>::Identity();
    Eigen::Matrix<double, Nx, Nx,RowMajor> A_sum = Eigen::Matrix<double, Nx, Nx, RowMajor>::Zero();

    for(int i=0; i<N+1; i++){
        A_sum = A_tmp + A_sum; 
        X_refined.block(Nx*i, 0, 6, 1) = A_tmp*Ax_Bu + A_sum*f_fix;
        A_tmp = A_tmp * A_fix; 
    }
    Eigen::Matrix<double, Nx*(N+1), 1> del_x = X_refined - ref; 

    Eigen::Matrix<double, -1, -1, RowMajor> f_cost(Nx*(N+1), 1);

    Eigen::Matrix<double, 1, Nx*(N+1)> transp_del_x = del_x.transpose();
    f_cost = transp_del_x*Q_dense*A_big*B_big; 
    
    
    qpOASES::real_t *Hqp = H_cost.data();
    qpOASES::real_t *g = f_cost.data();
    
    //Input, State Bound constraint 
    Eigen::Matrix<double, Nu*(N+1), 1> lbmat;
    Eigen::Matrix<double, Nu*(N+1), 1> ubmat;
    for(int i=0; i<N+1; ++i)
    {
        lbmat(Nu*i, 0) = jerk_min; ubmat(Nu*i, 0) = jerk_max;
        lbmat(Nu*i+1, 0) = steer_dot_min; ubmat(Nu*i+1, 0) = steer_dot_max;
    }

    qpOASES::real_t* lb = lbmat.data();
    qpOASES::real_t* ub = ubmat.data();

    qpOASES::real_t *solution = new qpOASES::real_t[Nu*(N+1)];
    qpOASES::int_t nWSR = 5000;
    
    qpOASES::Options options;
    options.printLevel = qpOASES::PL_LOW;
    options.terminationTolerance = 1e-10;
    

    qpOASES::QProblem initial_guess( Nu*(N+1), 0);
    initial_guess.setOptions(options);
    initial_guess.init(Hqp, g, NULL, lb, ub, NULL, NULL, nWSR, 0);
    
    if(initial_guess.isInfeasible()){
            cout<<"[QP solver] warning: problem is infeasible. "<<endl;
    }
    cout << "Acutual nWSR: " << nWSR << " / Original: " << 2000 << endl;
    
    initial_guess.getPrimalSolution(solution);
    bool isSolved = initial_guess.isSolved();
    if (isSolved)
        cout<<"[QP solver] Success!  "<<endl;
    else
        cout<<"[QP solver] Failure.  "<<endl;


    for(int i=0; i< N+1; i++)
    {    
        double u1 = solution[2*i];
        double u2 = solution[2*i+1];   
        VectorU u_(u1, u2);
        uN_[i] = u_;
    }

}


template<const int Nx, const int Nu, const int N>
Collection<Matrix<double,Nu,1>,N> QP<Nx,Nu,N>::get_solution( )
{
    return uN_;
}


#endif