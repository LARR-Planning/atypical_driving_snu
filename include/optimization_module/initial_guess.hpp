#ifndef INITIAL_GUESS_HPP
#define INITIAL_GUESS_HPP

#include <util.hpp>
#include <qpOASES.hpp>
#include <vector>
#include <iostream>
#include <Eigen/Core>
#include <optimization_module/dimension.h>
#include <optimization_module/parameters/dyn_parameter.h>

using namespace std;
// using namespace Eigen;

typedef Eigen::Triplet<double> Trip;

class QP_block
{
    private:
    double Q_; double R_;
        Collection<Matrix<double,5,1>,N+1> local_wpts;

    public:
    QP_block(Collection<Eigen::Matrix<double,5,1>,N+1> local_wpts_, double Q, double R):local_wpts(local_wpts_), Q_(Q), R_(R){}

    // qpOASES::real_t* QP_block_solver( Eigen::Matrix<double, Nx, 1> x_init, Eigen::Matrix<double, Nu, 1> u_init)
    // {

    //     Eigen::Matrix<double, Nx, Nx, RowMajor> A = symbolic_functions::dfdx(x_init, u_init);
    //     Eigen::Matrix<double, Nx, Nu, RowMajor> B = symbolic_functions::dfdu(x_init, u_init);
    //     Eigen::Matrix<double, Nx, 1> f0 = symbolic_functions::f(x_init, u_init);
    //     Eigen::Matrix<double, Nx, Nx, RowMajor> A_fix = A*dt + Eigen::Matrix<double, Nx, Nx, RowMajor>::Identity();
    //     Eigen::Matrix<double, Nx, Nu, RowMajor> B_fix = B*dt; 

    //     Eigen::Matrix<double, Nx, 1> f_fix = -(A*x_init*dt +B_fix*u_init) + f0 *dt;

    //     std::vector<Trip> trpQ; // Cost Matrix 
    //     std::vector<Trip> trpR; // Cost Matrix 
        
    //     std::vector<Trip> trpA; // A_eq
    //     std::vector<Trip> trpB; // B_eq 
    //     Trip tmp_A1;  Trip tmp_A2; Trip tmp_A3; Trip tmp_B1; Trip trmp_Q; Trip trmp_R;

    //     //Cost Matrix Generation 
    //     double Q = Q_; double R = R_; 
    //     for(int i=0; i<N+1; ++i){
    //         for(int j=0; j<2; ++j){
    //             trmp_Q = Trip(Nx*i+j, Nx*i+j, Q);
    //             trpQ.push_back(trmp_Q);
    //             trmp_R = Trip(Nu*i+j, Nu*i+j, R);
    //             trpR.push_back(trmp_R);
    //         }
    //     }
    //     Eigen::SparseMatrix<double, RowMajor> Q_sparse( (Nx)*(N+1), (Nx)*(N+1) );
    //     Q_sparse.setFromTriplets(trpQ.begin(), trpQ.end());

    //     Eigen::Matrix<double, -1, -1, RowMajor> Q_dense = MatrixXd(Q_sparse);

    //     Eigen::SparseMatrix<double, RowMajor> R_sparse(Nu*(N+1), Nu*(N+1));
    //     R_sparse.setFromTriplets(trpR.begin(), trpR.end());

    //     Eigen::Matrix<double, -1, -1, RowMajor> A_big = Eigen::Matrix<double, Nx*(N+1), Nx*(N+1),RowMajor>::Zero();
    //     Eigen::Matrix<double, -1, -1, RowMajor> B_big = Eigen::Matrix<double, Nx*(N+1), Nu*(N+1),RowMajor>::Zero();
        
    //     for(int i=0; i<N+1; i++)
    //     {
    //         Eigen::Matrix<double, Nx, Nx,RowMajor> A_tmp = A_fix.inverse();
    //         for(int j=i-1; j>=0; j--){
    //             A_tmp = A_tmp * A_fix;
    //             A_big.block(Nx*i, Nx*j, Nx, Nx) = A_tmp;
    //         }

    //         B_big.block(Nx*i, Nu*i, Nx, Nu) = B_fix; 
    //     }

    //     Eigen::Matrix<double, Nu*(N+1), Nu*(N+1), RowMajor> H_cost;
    //     H_cost = B_big.transpose()*A_big.transpose()*Q_sparse*A_big*B_big + R_sparse;
    //     H_cost = 1.0/2.0*(H_cost + H_cost.transpose());

    //     Eigen::Matrix<double, Nx*(N+1), 1> X_refined; 
    //     Eigen::Matrix<double, Nx, 1> Ax_Bu = A_fix*x_init + B_fix*u_init;
    //     Eigen::Matrix<double, Nx, Nx,RowMajor> A_tmp = Eigen::Matrix<double, Nx, Nx, RowMajor>::Identity();
    //     Eigen::Matrix<double, Nx, Nx,RowMajor> A_sum = Eigen::Matrix<double, Nx, Nx, RowMajor>::Zero();

    //     for(int i=0; i<N+1; i++){
    //         A_sum = A_tmp + A_sum; 
    //         X_refined.block(Nx*i, 0, 6, 1) = A_tmp*Ax_Bu + A_sum*f_fix;
    //         A_tmp = A_tmp * A_fix; 
    //     }
    //     Eigen::Matrix<double, Nx*(N+1), 1> del_x = X_refined - ref; 

    //     Eigen::Matrix<double, -1, -1, RowMajor> f_cost(Nx*(N+1), 1);

    //     Eigen::Matrix<double, 1, Nx*(N+1)> transp_del_x = del_x.transpose();
    //     f_cost = transp_del_x*Q_dense*A_big*B_big; 
        
        
    //     qpOASES::real_t *Hqp = H_cost.data();
    //     qpOASES::real_t *g = f_cost.data();
        
    //     //Input, State Bound constraint 
    //     Eigen::Matrix<double, Nu*(N+1), 1> lbmat;
    //     Eigen::Matrix<double, Nu*(N+1), 1> ubmat;
    //     for(int i=0; i<N+1; ++i)
    //     {
    //         lbmat(Nu*i, 0) = jerk_min; ubmat(Nu*i, 0) = jerk_max;
    //         lbmat(Nu*i+1, 0) = steer_dot_min; ubmat(Nu*i+1, 0) = steer_dot_max;
    //     }

    //     qpOASES::real_t* lb = lbmat.data();
    //     qpOASES::real_t* ub = ubmat.data();

    //     qpOASES::real_t *x1 = new qpOASES::real_t[Nu*(N+1)];
    //     qpOASES::int_t nWSR = 5000;
        
    //     qpOASES::Options options;
    //     options.printLevel = qpOASES::PL_LOW;
    //     options.terminationTolerance = 1e-10;
        

    //     qpOASES::QProblem initial_guess( Nu*(N+1), 0);
    //     initial_guess.setOptions(options);
    //     initial_guess.init(Hqp, g, NULL, lb, ub, NULL, NULL, nWSR, 0);
        
    //     if(initial_guess.isInfeasible()){
    //             cout<<"[QP solver] warning: problem is infeasible. "<<endl;
    //     }
    //     cout << "Acutual nWSR: " << nWSR << " / Original: " << 2000 << endl;
        
    //     initial_guess.getPrimalSolution(x1);
    //     bool isSolved = initial_guess.isSolved();
    //     if (isSolved)
    //         cout<<"[QP solver] Success!  "<<endl;
    //     else
    //         cout<<"[QP solver] Failure.  "<<endl;
            
    //     return x1;
    // }
};

#endif