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

void ref_traj(Collection<Matrix<double,5,1>,N+1>& local_wpts, Collection<Matrix<double,5,1>,N+1>& waypt, double initial_vel)
{   
    initial_vel = max(initial_vel, 0.5);// Prevent vel = 0.0
    std::vector<double> length_vec;
    double dist = 0.0;
    length_vec.push_back(dist);
    for(int i=1; i<N+1; i++){
        dist = dist + sqrt((waypt[i-1][0] - waypt[i][0])*(waypt[i-1][0] - waypt[i][0]) + (waypt[i-1][1] - waypt[i][1])*(waypt[i-1][1] - waypt[i][1]));
        length_vec.push_back(dist);
    }

    int iter = 0;
    for(double t=0.0; t<5.0; t=t+dt){
        int idx = 0;
        double cur_s = t* initial_vel; 
            while(idx+1 < length_vec.size() && !(cur_s >=length_vec.at(idx) && cur_s <=length_vec.at(idx+1))){
                idx++;
            }
            double len = waypt[idx+1][0] - waypt[idx][0];
            cur_s = cur_s - length_vec.at(idx);
            double x1 = waypt[idx][0]; double x2 = waypt[idx+1][0];
            double y1 = waypt[idx][1]; double y2 = waypt[idx+1][1];
            double dsdx = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2))/(x2-x1);

            double dx = cur_s/dsdx; 
            double x = x1 + dx; double y = y1+(y2-y1)*dx/len; 

            local_wpts[iter][0] = x;
            local_wpts[iter][1] = y;
            local_wpts[iter][2] = 0.0; local_wpts[iter][3] = 0.0; local_wpts[iter][4] = 0.0; 
            iter++;
    }

}


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
        Collection<Matrix<double,5,1>,N+1>& local_wpts_, 
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
                Collection<Matrix<double,5,1>,N+1>& local_wpts_, 
                Vector2d qp_param)
                 : prob_(prob), local_wpts(local_wpts_)
{
    // initial setups
	x0_ = x_init;
	// xN_[0] = x0_;
	u0_ = u_init;
	dt_ = dt;
    Q_ = qp_param(0,0); R_ = qp_param(1,0);
    // cout << "Q: " << Q_ << " R: " << R_ << endl; 

}


template<const int Nx, const int Nu, const int N>
void QP<Nx,Nu,N>::solve( )
{   
    //Change for QP. Too many variables exceed planning time. 
    const double dt_ = 0.2; 
    const int N_ = 25;
    Eigen::Matrix<double, Nx*(N_+1),1> ref;
    // cout << "Reference pts." << endl; 

    Collection<Matrix<double,5,1>,N+1> tf_wpts; 
    double initial_vel = x0_(2);
    ref_traj(tf_wpts, local_wpts, initial_vel);

    
    for(int i=0; i<N_+1; i++)
    {
        ref(6*i,0) = local_wpts[2*i][0];
        ref(6*i+1,0) = local_wpts[2*i][1]; 
        ref(6*i+2,0) = local_wpts[2*i][2];
        ref(6*i+3,0) = local_wpts[2*i][3];
        ref(6*i+4,0) = local_wpts[2*i][4];
        ref(6*i+5,0) = 0.0; 
        // cout << tf_wpts[2*i][0] << ", " << tf_wpts[2*i][1] << endl; 
        // cout << local_wpts[2*i][0] << ", " << local_wpts[2*i][1] << endl; 
    }

    DynamicsDerivatives<Nx,Nu> dyn = prob_.dynamics(x0_, u0_, dt_);
    //dyn.fx -> A_fix (A*dt+I)
    //dyn.fu -> B_fix (B*dt)

    Eigen::Matrix<double, Nx, Nx, RowMajor> A = symbolic_functions::dfdx(x0_, u0_);
    Eigen::Matrix<double, Nx, Nu, RowMajor> B = symbolic_functions::dfdu(x0_, u0_);
    Eigen::Matrix<double, Nx, 1> f0 = symbolic_functions::f(x0_, u0_);
    // Eigen::Matrix<double, Nx, Nx, RowMajor> A_fix = A*dt + Eigen::Matrix<double, Nx, Nx, RowMajor>::Identity();
    Eigen::Matrix<double, Nx, Nx, RowMajor> A_fix = A*dt_ + Eigen::Matrix<double, Nx, Nx, RowMajor>::Identity();
    // Eigen::Matrix<double, Nx, Nu, RowMajor> B_fix = B*dt; 
    Eigen::Matrix<double, Nx, Nu, RowMajor> B_fix = B*dt_; 

    Eigen::Matrix<double, Nx, 1> f_fix = -(A*x0_*dt_ +B_fix*u0_) + f0 *dt_;

    std::vector<Trip> trpQ; // Cost Matrix 
    std::vector<Trip> trpR; // Cost Matrix 
    
    std::vector<Trip> trpA; // A_eq
    std::vector<Trip> trpB; // B_eq 
    Trip tmp_A1;  Trip tmp_A2; Trip tmp_A3; Trip tmp_B1; Trip trmp_Q; Trip trmp_R;

    // cout << "A_fix" << endl;
    // cout << A_fix << endl; 

    // cout << "B_fix" << endl; cout << B_fix << endl;  
    //Cost Matrix Generation 
    double Q = Q_; double R = R_; 
    for(int i=0; i<N_+1; ++i){
        for(int j=0; j<2; ++j){
            trmp_Q = Trip(Nx*i+j, Nx*i+j, Q);
            trpQ.push_back(trmp_Q);
            trmp_R = Trip(Nu*i+j, Nu*i+j, R);
            trpR.push_back(trmp_R);
        }
    }
    Eigen::SparseMatrix<double, RowMajor> Q_sparse( (Nx)*(N_+1), (Nx)*(N_+1) );
    Q_sparse.setFromTriplets(trpQ.begin(), trpQ.end());

    Eigen::Matrix<double, -1, -1, RowMajor> Q_dense = MatrixXd(Q_sparse);

    Eigen::SparseMatrix<double, RowMajor> R_sparse(Nu*(N_+1), Nu*(N_+1));
    R_sparse.setFromTriplets(trpR.begin(), trpR.end());

    Eigen::Matrix<double, -1, -1, RowMajor> A_big = Eigen::Matrix<double, Nx*(N_+1), Nx*(N_+1),RowMajor>::Zero();
    Eigen::Matrix<double, -1, -1, RowMajor> B_big = Eigen::Matrix<double, Nx*(N_+1), Nu*(N_+1),RowMajor>::Zero();
    
    for(int i=0; i<N_+1; i++)
    {
        Eigen::Matrix<double, Nx, Nx,RowMajor> A_tmp = A_fix.inverse();
        for(int j=i-1; j>=0; j--){
            A_tmp = A_tmp * A_fix;
            A_big.block(Nx*i, Nx*j, Nx, Nx) = A_tmp;
        }

        B_big.block(Nx*i, Nu*i, Nx, Nu) = B_fix; 
    }

    Eigen::Matrix<double, Nu*(N_+1), Nu*(N_+1), RowMajor> H_cost;
    H_cost = B_big.transpose()*A_big.transpose()*Q_sparse*A_big*B_big + R_sparse;
    H_cost = 1.0/2.0*(H_cost + H_cost.transpose());


    // cout << "H_matrix" << endl; cout << H_cost << endl; 

    Eigen::Matrix<double, Nx*(N_+1), 1> X_refined; 
    Eigen::Matrix<double, Nx, 1> Ax_Bu = A_fix*x0_ + B_fix*u0_;
    Eigen::Matrix<double, Nx, Nx,RowMajor> A_tmp = Eigen::Matrix<double, Nx, Nx, RowMajor>::Identity();
    Eigen::Matrix<double, Nx, Nx,RowMajor> A_sum = Eigen::Matrix<double, Nx, Nx, RowMajor>::Zero();

    for(int i=0; i<N_+1; i++){
        A_sum = A_tmp + A_sum; 
        X_refined.block(Nx*i, 0, 6, 1) = A_tmp*Ax_Bu + A_sum*f_fix;
        A_tmp = A_tmp * A_fix; 
    }
    Eigen::Matrix<double, Nx*(N_+1), 1> del_x = X_refined - ref; 

    Eigen::Matrix<double, -1, -1, RowMajor> f_cost(Nx*(N_+1), 1);

    Eigen::Matrix<double, 1, Nx*(N_+1)> transp_del_x = del_x.transpose();
    f_cost = transp_del_x*Q_dense*A_big*B_big; 
    

    
    qpOASES::real_t *Hqp = H_cost.data();
    qpOASES::real_t *g = f_cost.data();
    
    //Input, State Bound constraint 
    Eigen::Matrix<double, Nu*(N_+1), 1> lbmat;
    Eigen::Matrix<double, Nu*(N_+1), 1> ubmat;
    for(int i=0; i<N_+1; ++i)
    {
        lbmat(Nu*i, 0) = jerk_min-3.0; ubmat(Nu*i, 0) = jerk_max+3.0;
        lbmat(Nu*i+1, 0) = steer_dot_min; ubmat(Nu*i+1, 0) = steer_dot_max;
    }

    qpOASES::real_t* lb = lbmat.data();
    qpOASES::real_t* ub = ubmat.data();

    qpOASES::real_t *solution = new qpOASES::real_t[Nu*(N_+1)];
    qpOASES::int_t nWSR = 5000;
    
    qpOASES::Options options;
    options.printLevel = qpOASES::PL_LOW;
    options.terminationTolerance = 1e-10;
    

    qpOASES::QProblem initial_guess( Nu*(N_+1), 0);
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

    // cout << "Solution" << endl; 

    for(int i=0; i< N_+1; i++)
    {    
        double u1 = solution[2*i];
        double u2 = solution[2*i+1];   
        // cout << u1 << ", " << u2 << endl; 

        VectorU u_(u1, u2);
        uN_[2*i] = u_;
        uN_[2*i+1] = u_;
        // uN_[i] = u_;
    
    }

}


template<const int Nx, const int Nu, const int N>
Collection<Matrix<double,Nu,1>,N> QP<Nx,Nu,N>::get_solution( )
{
    return uN_;
}


#endif