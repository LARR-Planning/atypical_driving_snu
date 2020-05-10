#ifndef PROBLEM_HPP
#define PROBLEM_HPP

#include <optimization_module/system_base_atypical.h>
#include <optimization_module/problem_description.h>
#include <optimization_module/parameters/dyn_parameter.h>
#include <atypical_planner/PlannerCore.h>

#include <optimization_module/dimension.h>
#include <optimization_module/symbolic_functions/f.hpp>
#include <optimization_module/symbolic_functions/dfdx.hpp>
#include <optimization_module/symbolic_functions/dfdu.hpp>
#include <optimization_module/symbolic_functions/cost.hpp>
#include <optimization_module/symbolic_functions/costx.hpp>
#include <optimization_module/symbolic_functions/costxx.hpp>
#include <optimization_module/symbolic_functions/costu.hpp>
#include <optimization_module/symbolic_functions/costuu.hpp>



/* YW added*/
template<typename T, const int size>
using Collection = std::array<T, size>; 

class Problem : public ProblemDescription<Nx,Nu>
{
	private:

		//SystemBase<Nx,Nu>& sys;
        Matrix<double,2,1> x_goal_; // Goal : the last point of last corridor

		Matrix<double,2,2> Qx; // shape matrix, constant in plain_MPC


        VectorX final_weight_ = VectorX::Zero(); //Final Cost Weight Factor
        VectorX state_weight_ = VectorX::Zero(); //Running Cost State Weight Factor
		VectorU input_weight_ = VectorU::Zero(); //Running Cost Input Weight Factor

		bool obs_computed_ = true;
        bool use_target_ = false;

        //Collection<Collection<Matrix<double,2,1>,N+1>,N_obs> obs_q_;
		//Collection<Collection<Matrix<double,2,2>,N+1>,N_obs> obs_Q_;
		vector<vector<Matrix<double,2,2>>> obs_Q_; //Out: step In: obstacle instances
		vector<vector<Matrix<double,2,1>>> obs_q_; //Out: step In: obstacle instances
		Collection<Planner::Corridor,N+1>& sfc_modified;

        bool noConstraint_ = true;

	public:
        Problem (Collection<Planner::Corridor,N+1>&corridor_seq)
        : sfc_modified(corridor_seq)
        { }
		void set_goal( const Matrix<double,2,1> x_goal)
		{ x_goal_ = x_goal; }
		
		void set_input_weight( const VectorU weight )
		{ input_weight_ = weight;	}
		
		void set_state_weight( const VectorX weight )
		{ state_weight_ = weight; }

		void set_final_weight( const VectorX weight )
		{ final_weight_ = weight;	}

		void set_noConstraint( const bool noConstraint )
		{ noConstraint_ = noConstraint; }

//		void set_constraintype( const int constraintType )
//		{ constraintType_ = constraintType; }
        /*
		void set_obstacle( const Matrix<double,3,1> q,
							const Matrix<double,3,3> Q )
		{ 	q_obs.push_back( q );	Q_obs.push_back( Q ); }
		
        void clear_obstacles()
		{	q_obs.clear(); Q_obs.clear(); }
        */
        DynamicsDerivatives<Nx,Nu>
            dynamics(const VectorX x, const VectorU u, const double dt, const ReturnType type)
        {
            // simple euler integration
            DynamicsDerivatives<Nx,Nu> dyn;
            dyn.f = x +dt*symbolic_functions::f(x,u);
            if (type == WITHOUT_DERIVATIVES)
                return dyn;
            // fx and fu should be based on discretized f!!!
            dyn.fx = MatrixXX::Identity()+ dt*symbolic_functions::dfdx(x,u);
            dyn.fu = dt*symbolic_functions::dfdu(x,u);
            return dyn;            
        }         

        CostDerivatives<Nx,Nu>
        cost(const VectorX x, const VectorU u, const int idx, const ReturnType type)
        {
            if(!std::isnan(u(0)))
            {
                // running cost must be computed here
                CostDerivatives<Nx,Nu> obj;
                obj.c = symbolic_functions::cost(x,u,state_weight_,input_weight_,x_goal_,obs_q_[idx][0]);
                if( type == WITHOUT_DERIVATIVES)
                    return obj;
                obj.cx = symbolic_functions::costx(x,u,state_weight_,input_weight_,x_goal_,obs_q_[idx][0]);
				obj.cu = symbolic_functions::costu(x,u,state_weight_,input_weight_,x_goal_,obs_q_[idx][0]);
				obj.cxx = symbolic_functions::costxx(x,u,state_weight_,input_weight_,x_goal_, obs_q_[idx][0]);
				obj.cxu = MatrixXU::Zero();					
				obj.cuu = symbolic_functions::costuu(x,u,state_weight_,input_weight_,x_goal_,obs_q_[idx][0]);
                return obj;
            }
            else
            {
                // final cost must be computed here
                Matrix<double,Nu,1> u_Zero = Matrix<double,Nu,1>::Zero();
				CostDerivatives<Nx,Nu> obj;
                obj.c = symbolic_functions::cost(x,u_Zero,final_weight_,input_weight_,x_goal_,obs_q_[idx][0]);
                if( type == WITHOUT_DERIVATIVES)
                    return obj;
                obj.cx = symbolic_functions::costx(x,u_Zero,final_weight_,input_weight_,x_goal_,u_Zero);
				obj.cu = VectorU::Zero();
				obj.cxx = symbolic_functions::costxx(x,u_Zero,final_weight_,input_weight_,x_goal_,u_Zero);
				obj.cxu = MatrixXU::Zero();				
				obj.cuu = MatrixUU::Zero();

                return obj;
            }
        }

        int idx_const_par = 0;

        ConstraintDerivatives<Nx,Nu>
            constraint(const VectorX x, const VectorU u, const int idx, const ReturnType type)
            {
                if(idx >= N)
                {
                    idx_const_par = N-1;
                }
                else
                {
                    idx_const_par = idx;
                }

                int nieq = Nc;
                int neq = 0;
                int N_obs = obs_q_[0].size();
                Matrix<double,5,1> x_ = x.block<5,1>(0,0);
                Matrix<double,2,1> u_ = u.block<2,1>(0,0);
                
                ConstraintDerivatives<Nx,Nu> obj(10,0);
                Matrix<double,2,1> q1 = x_.block<2,1>(0,0);
                Matrix<double,2,2> Q1 = Qx.block<2,2>(0,0);
                double sqrtTrQ1 = sqrt(Q1.trace());
                
                // Matrix<double,2,1> q2 = obs_q_[idx](i);
                // Matrix<double,2,2> Q2 = obs_Q_[idx](i);
                for(int i = 0;i< N_obs ;i++)
                {
                    Matrix<double,2,1> q2 = obs_q_[idx][i];
                    Matrix<double,2,2> Q2 = obs_Q_[idx][i];

                    double sqrtTrQ2 = sqrt(Q2.trace());
                    Matrix<double,2,2> Qsum = (1.0+sqrtTrQ2/sqrtTrQ1)*Q1 
                                        + (1.0+sqrtTrQ1/sqrtTrQ2)*Q2;
                    Matrix<double,2,2> invQsum = Qsum.inverse();
                    obj.con(i) = (q1-q2).dot( invQsum*(q1-q2) ) - 1.0;

                    if(type != WITHOUT_DERIVATIVES)
                    {
                        obj.conx.row(i).block<1,2>(0,0) = 2.0*(invQsum*(q1-q2)).transpose();
                    }

                }
                // SFC constraints
                obj.con(2) = x_(0) - sfc_modified[idx].xl;
                obj.con(3) = sfc_modified[idx].xu - x_(0);
                obj.con(4) = x_(1) - sfc_modified[idx].yl;
                obj.con(5) = sfc_modified[idx].yu - x_(1);
                obj.con(6) = x_(3) - steer_min;
                obj.con(7) = steer_max - x_(3);
                obj.con(8) = u_(0) - acc_min;
                obj.con(9) = acc_max - u_(0);                  
                
                if(type != WITHOUT_DERIVATIVES)
                {
                    obj.conx.row(2).coeffRef(0,0) = 1.0;
                    obj.conx.row(3).coeffRef(0,0) = -1.0;
                    obj.conx.row(4).coeffRef(0,1) = 1.0;
                    obj.conx.row(5).coeffRef(0,1) = -1.0;
                    obj.conx.row(6).coeffRef(0,3) = 1.0;
                    obj.conx.row(7).coeffRef(0,3) = -1.0;

                    obj.conu.row(8).coeffRef(0,0) = 1.0;
                    obj.conu.row(9).coeffRef(0,1) = -1.0;
                }
                return obj;
                    // if(type == WITHOUT_DERIVATIVES)
                    //     return obj;
                    // obj.conx.row(i).block<1,2>(0,0) = 2.0*(invQsum*(q1-q2)).transpose();
                    // return obj;
            }
};
#endif