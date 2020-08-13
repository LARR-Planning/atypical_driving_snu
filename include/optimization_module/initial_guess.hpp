#ifndef INITIAL_GUESS_HPP
#define INITIAL_GUESS_HPP

#include <Eigen/Core>
#include <optimization_module/dimension.h>
#include <optimization_module/parameters/dyn_parameter.h>
#include <util.hpp>
#include <qpOASES.hpp>

using namespace std;
using namespace Eigen;

typedef Triplet<double> Trip;

struct triplet{
    int row; int col; double val;
    triplet(int row_, int col_, double val_) : row(row_), col(col_), val(val_){} 
};

//True if t1 < t2. False if t2 < t1. 
bool compare(triplet& t1, triplet& t2){
    if(t1.col>t2.col)
        return false;
    else if(t1.col < t2.col)
        return true;
    else{
        if(t1.row > t2.row)
            return false;    
        else
        {
            return true; 
        }
    }
}
void Print(std::vector<triplet> t_vec){
    for(int i=0; i<t_vec.size(); ++i){
        std::cout << t_vec.at(i).row << " " << t_vec.at(i).col << " " << t_vec.at(i).val << std::endl;
    }
}

class QP_block
{
    private:
        double EPS = 1e-2;
        qpOASES::sparse_int_t H_ir[(2+Nu)*(N+1)];
        qpOASES::sparse_int_t H_jc[(Nx+Nu)*(N+1)];
        qpOASES::real_t H_val[(2+Nu)*(N+1)];
        Collection<Matrix<double,5,1>,N+1> local_wpts;

        double Q; 
        double R;

    public:
    QP_block(Collection<Matrix<double,5,1>,N+1> local_wpts_, double Qinput, double Rinput)
    :local_wpts(local_wpts_), Q(Qinput), R(Rinput)
    {}

    std::pair<std::vector<triplet>, std::vector<Trip>> A_gen(Eigen::Matrix<double, Nx, Nx, RowMajor> A_fix, 
                                                            Eigen::Matrix<double, Nx, Nu, RowMajor> B_fix, 
                                                            Eigen::Matrix<double, Nx, 1> f_fix,
                                                            Eigen::Matrix<double, Nx, 1> x_init)
    {
        std::vector<triplet> trpA; // A_eq
        std::vector<Trip> trpB; // B_eq 
        triplet tmp_A1 = triplet(0, 0, 0.0);  triplet tmp_A2 = triplet(0, 0, 0.0); 
        triplet tmp_A3 = triplet(0, 0, 0.0); 
        Trip tmp_B1;

        for(int i=0; i<N; i++){
            for(int j=0; j<Nx; j++){
                for(int k=0; k<Nx; k++){
                    if(abs(A_fix(j,k))>EPS)
                    {
                        tmp_A1.row = Nx*(i+1)+j;
                        tmp_A1.col = Nx*i+k;
                        tmp_A1.val = -1.0*A_fix(j,k);
                        trpA.push_back(tmp_A1);
                    }
                    if(j==k){
                        tmp_A2.row = Nx*(i+1)+j;
                        tmp_A2.col = Nx*(i+1)+k;
                        tmp_A2.val = 1.0;
                        trpA.push_back(tmp_A2);
                    }
                }
                for(int k=0; k<Nu; k++){
                    if(abs(B_fix(j,k))>EPS){
                        tmp_A3.row = Nx*(i+1)+j;
                        tmp_A3.col = Nx*(N+1) + Nu*i+k;
                        tmp_A3.val = -1.0*B_fix(j,k);
                        trpA.push_back(tmp_A3);
                    }
                }
                
                if(abs(f_fix(j,0))>EPS){
                    tmp_B1 = Trip(Nx*(i+1)+j, 0, -1.0 * f_fix(j,0));
                    trpB.push_back(tmp_B1);
                }
            }
        }
        //Initial Condition
        for(int i=0; i<Nx; i++){
            tmp_A1.row = i; tmp_A1.col = i; tmp_A1.val = 1.0;
            trpA.push_back(tmp_A1);
            tmp_B1 = Trip(i, 0, x_init(i,0));
            trpB.push_back(tmp_B1);
        }
        //A matrix
        std::sort(trpA.begin(), trpA.end(), compare);
        std::pair<std::vector<triplet>, std::vector<Trip>> return_pair = std::make_pair(trpA, trpB);
        return return_pair;
    }

    qpOASES::real_t* QP_block_solver( Eigen::Matrix<double, Nx, 1> x_init, Eigen::Matrix<double, Nu, 1> u_init)
    {
        Eigen::Matrix<double, Nx, Nx, RowMajor> A = symbolic_functions::dfdx(x_init, u_init);
        Eigen::Matrix<double, Nx, Nu, RowMajor> B = symbolic_functions::dfdu(x_init, u_init);
        Eigen::Matrix<double, Nx, 1> f0 = symbolic_functions::f(x_init, u_init);
        Eigen::Matrix<double, Nx, Nx, RowMajor> A_fix = A*dt + Eigen::Matrix<double, Nx, Nx, RowMajor>::Identity();
        Eigen::Matrix<double, Nx, Nu, RowMajor> B_fix = B*dt; 
        Eigen::Matrix<double, Nx, 1> f_fix = A*x_init*dt +B_fix*u_init - f0 *dt;

        //Cost Matrix Generation 
        std::vector<Trip> trpH; // Cost Matrix
        Trip tmp_H1; Trip tmp_H2;

        //Cost Matrix Generation 
        double Q = 100.0; double R = 1.0; 
        for(int i=0; i<N+1; ++i){
            for(int j=0; j<2; ++j){
                tmp_H1 = Trip(Nx*i+j, Nx*i+j, Q);
                trpH.push_back(tmp_H1);
                tmp_H2 = Trip(Nx*(N+1)+Nu*i+j, Nx*(N+1)+Nu*i+j, R);
                trpH.push_back(tmp_H2);
            }
        }
        Eigen::SparseMatrix<double, RowMajor> H_sparse( (Nx+Nu)*(N+1), (Nx+Nu)*(N+1) );
        H_sparse.setFromTriplets(trpH.begin(), trpH.end());

        //H Matrix 
        
        int iter_ir = 0; int iter_jr = 0; int iter_val = 0;
        for(int i=0; i< (Nx+Nu)*(N+1); i++){
            if(i < Nx*N){
                if(i%6==0 || i%6==1)
                {   H_ir[iter_ir] = i; 
                    H_jc[iter_jr] = iter_ir; 
                    H_val[iter_val] = Q; 
                    iter_ir++; iter_jr++; iter_val++;
                }
                else{
                    H_jc[iter_jr] = iter_ir; 
                    iter_jr++;
                }
            }
            else{
                H_ir[iter_ir] = i; 
                H_jc[iter_jr] = iter_ir; 
                H_val[iter_val] = R; 
                iter_ir++; iter_jr++; iter_val++;
            }
        }

        //ir, val. -> ir's k-th entry: k-th row of k-th element of val[]
        //jc -> Each column, Index in val[] of the first number appeared in the column
        qpOASES::SymSparseMat *H_sp = new qpOASES::SymSparseMat((Nx+Nu)*(N+1),(Nx+Nu)*(N+1), H_ir, H_jc, H_val);
        // qpOASES::SparseMatrix *A_sp = new SparseMatrix(5, 5, A_ir, A_jc, A_val);
        
        H_sp->createDiagInfo();


        //F matrix generation 
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

        Eigen::Matrix<double, Nu*(N+1), 1> zeros = Eigen::Matrix<double, Nu*(N+1), 1>::Zero();
        Eigen::Matrix<double, (Nx+Nu)*(N+1), 1> append_ref;  append_ref << ref, zeros; 
        Eigen::Matrix<double, -1, -1, RowMajor> f_cost((Nx+Nu)*(N+1), 1);

        f_cost = -2*H_sparse*append_ref;


        //Constraint Matrix Generation 
        std::pair<std::vector<triplet>, std::vector<Trip>> pair_AB = A_gen(A_fix, B_fix, f_fix, x_init);
        std::vector<triplet> trpA = pair_AB.first; // A_eq
        std::vector<Trip> trpB = pair_AB.second;

        // Print(trpA);
        int size_row = trpA.size();
        int size_col = (Nx+Nu)*(N+1);
        int size_val = size_row;
        qpOASES::sparse_int_t A_ir[size_row];
        qpOASES::sparse_int_t A_jc[size_col];
        qpOASES::real_t A_val[size_val];  
        
        int iter_A = 0; int iter_row = 0; iter_val = 0;
        for(int j=0; j<size_col; ++j){
            //If j'th column is not in A_jc array, then skip. 
            if(iter_A >= trpA.size())
            {   cout << "BREAK! " << j << endl;
                A_jc[j] = iter_A;
            }
            else{
                if( j < trpA.at(iter_A).col)
                    {
                        A_jc[j] = iter_A;
                    }
                else{
                    // j = trpA.at(iter_A).col 
                    int start_idx = iter_A;
                    int end_idx = iter_A;

                    while(end_idx < trpA.size() && trpA.at(end_idx).col ==j){    
                        end_idx++;
                        // if(end_idx > trpA.size())
                        //     break; 
                        std::cout << "end_idx: " <<end_idx << " j: " << j << std::endl;
                    }
                    A_jc[j] = iter_A;
                    std::vector<triplet>::iterator first = trpA.begin()+start_idx;
                    std::vector<triplet>::iterator end = trpA.begin()+end_idx;
                    for(std::vector<triplet>::iterator it = first; it!=end; it++){
                        auto triplet_val = *it; 
                        A_ir[iter_row] = triplet_val.row;
                        A_val[iter_val] = triplet_val.val; 
                        iter_row++; iter_val++;
                    }
                    
                    iter_A = end_idx; 
                }
            }
        }
        // cout << iter_ir << " " << iter_jr << " " << iter_val << endl;
        int tot = (Nx+Nu)*(N+1); 
        qpOASES::SparseMatrix *A_sp = new qpOASES::SparseMatrix(tot, tot, A_ir, A_jc, A_val);
        A_sp->createDiagInfo();

        //B matrix 
        Eigen::SparseMatrix<double, RowMajor> B_sparse((Nx+Nu)*(N+1), 1);
        B_sparse.setFromTriplets(trpB.begin(), trpB.end());
        Eigen::Matrix<double, -1, -1,RowMajor> B_eq; 
        B_eq = Eigen::MatrixXd(B_sparse);

        Eigen::Matrix<double, 1, -1, RowMajor> gmat((f_cost.transpose()).template cast<double>());
        Eigen::Matrix<double, -1, -1, RowMajor> lbAmat(B_eq.template cast<double>());
        
        // qpOASES::real_t *Hqp = Hmat.data();
        // qpOASES::real_t *Aqp = Amat.data();
        qpOASES::real_t *g = gmat.data();
        qpOASES::real_t *lbA = lbAmat.data();
        qpOASES::real_t *ubA = lbAmat.data();
        

        //Input, State Bound constraint 
        const int total_N = (Nx+Nu)*(N+1);
        // qpOASES::real_t *lb = new qpOASES::real_t[total_N];
        // qpOASES::real_t *ub = new qpOASES::real_t[total_N];
        Eigen::Matrix<double, 1, total_N, RowMajor> lbmat;
        Eigen::Matrix<double, 1, total_N, RowMajor> ubmat;
        // qpOASES::real_t lb[total_N];
        // qpOASES::real_t ub[total_N];
        for(int i=0; i<total_N; ++i)
        {
            if(i < Nx*(N+1)){ //State bound 
                if(i%6==3){
                    lbmat(0,i) = acc_min; ubmat(0,i) = acc_max;
                    // lbmat(0,i) = -100.0; ubmat(0,i) = 100.0;

                }
                else if(i%6==4){
                    lbmat(0,i) = steer_min; ubmat(0,i) = steer_max; 
                    // lbmat(0,i) = -100.0; ubmat(0,i) = 100.0;
                }
                else{
                    lbmat(0,i) = -1000; ubmat(0,i) = 1000;
                }
            }
            else{   //Input bound
                if(i%2==0){
                    // lbmat(0,i) = -100.0; ubmat(0,i) = 100.0;
                    lbmat(0,i) = jerk_min; ubmat(0,i) = jerk_max;
                }
                else if(i%2==1){
                    lbmat(0,i) = steer_dot_min; ubmat(0,i) = steer_dot_max;
                    // lbmat(0,i) = -100.0; ubmat(0,i) = 100.0;

                }
            }
        }

        qpOASES::real_t* lb = lbmat.data();
        qpOASES::real_t* ub = ubmat.data();

        qpOASES::real_t *x1 = new qpOASES::real_t[total_N];
        qpOASES::int_t nWSR = 2000;
        
        qpOASES::Options options;
        // options.printLevel = qpOASES::PL_LOW;
        options.terminationTolerance = 1e-10;
        

        qpOASES::QProblem initial_guess( total_N, total_N);
        initial_guess.setOptions(options);
        initial_guess.init(H_sp, g, A_sp, lb, ub, lbA, ubA, nWSR, 0);
        
        if(initial_guess.isInfeasible()){
                cout<<"[QP solver] warning: problem is infeasible. "<<endl;
        }
        cout << "Acutual nWSR: " << nWSR << " / Original: " << 2000 << endl;
        // qpOASES::QProblemB initial_guess(total_N);
        // initial_guess.init(Hqp, g, lb, ub, nWSR, 0);
        
        std::cout << " " << std::endl; 
        initial_guess.getPrimalSolution(x1);
        bool isSolved = initial_guess.isSolved();
        if (isSolved)
            cout<<"[QP solver] Success!  "<<endl;
        else
            cout<<"[QP solver] Failure.  "<<endl;

        // for(int i=0; i< N+1; i++)
        // {   
        //     cout << "X: " << x1[6*i]  << " Y: " << x1[6*i+1] << " v: " << x1[6*i+2] << " theta: " <<x1[6*i+5] << endl;

        //     cout << "jerk: " << x1[6*(N+1)+2*i] << " steer_dot: " << x1[6*(N+1)+2*i+1] << endl; 
        // }


        delete H_sp;
        delete A_sp;

        return x1;
    }
};

#endif