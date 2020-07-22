#ifndef PROBLEM_HPP
#define PROBLEM_HPP

//#include <optimization_module/system_base_atypical.h>
#include <optimization_module/problem_description.h>
#include <optimization_module/parameters/dyn_parameter.h>
#include <atypical_planner/PlannerCore.h>

#include <optimization_module/dimension.h>
#include <optimization_module/symbolic_functions/f.hpp>
#include <optimization_module/symbolic_functions/dfdx.hpp>
#include <optimization_module/symbolic_functions/dfdu.hpp>
#include <optimization_module/symbolic_functions/f_front.hpp>
#include <optimization_module/symbolic_functions/dfdx_front.hpp>
#include <optimization_module/symbolic_functions/dfdu_front.hpp>

#include <optimization_module/symbolic_functions/cost.hpp>
#include <optimization_module/symbolic_functions/costx.hpp>
#include <optimization_module/symbolic_functions/costxx.hpp>
#include <optimization_module/symbolic_functions/costu.hpp>
#include <optimization_module/symbolic_functions/costuu.hpp>
#include <optimization_module/symbolic_functions/con.hpp>
#include <optimization_module/symbolic_functions/conx.hpp>
#include <optimization_module/symbolic_functions/conu.hpp>
#include <optimization_module/symbolic_functions/con_final.hpp>
#include <optimization_module/symbolic_functions/conx_final.hpp>
#include <cmath>
#include <optimization_module/symbolic_functions/con_exp.hpp>
/* YW added*/
template<typename T, const int size>
using Collection = std::array<T, size>;

class Problem : public ProblemDescription<Nx,Nu>
{
private:

    Collection<Matrix<double,3,1>,N+1> x_ref; // Reference Tracking Mode
    int isRefUsed;
    VectorX final_weight_ = VectorX::Zero(); //Final Cost Weight Factor
    VectorX state_weight_ = VectorX::Zero(); //Running Cost State Weight Factor
    VectorU input_weight_ = VectorU::Zero(); //Running Cost Input Weight Factor

    Collection<Planner::Corridor,N+1>& sfc_modified;
    Collection<bool,N+1> sfc_idx;
    Matrix<double,2,2> car_shape;

    vector<vector<Matrix2d>>& obs_Q;
    vector<vector<Vector2d>>& obs_q;

    bool noConstraint_ = true;
    bool isRearWheel_;

public:
    Problem (Collection<Planner::Corridor,N+1>& corridor_seq, vector<vector<Vector2d>>& obs_pos,
            vector<vector<Matrix2d>>& obs_shape)
            : sfc_modified(corridor_seq), obs_Q(obs_shape),obs_q(obs_pos)
    { }

    void set_input_weight( const VectorU weight )
    { input_weight_ = weight;	}

    void set_state_weight( const VectorX weight )
    { state_weight_ = weight; }
    void set_car_shape(const Matrix<double,2,2> shape)
    { car_shape = shape; }

    void set_final_weight( const VectorX weight )
    { final_weight_ = weight;	}

    void set_ref(const Collection<Matrix<double,3,1>,N+1> x_ref_)
    {x_ref = x_ref_;}

    void set_sfc_idx(const Collection<bool,N+1> sfc_idx_)
    {sfc_idx = sfc_idx; }

    void set_refUsed(const int isRefUsed_)
    {isRefUsed = isRefUsed_;}

    void set_noConstraint( const bool noConstraint )
    { noConstraint_ = noConstraint; }

    void setRear_wheel(const bool isRearWheel)
    { isRearWheel_ = isRearWheel;}


    DynamicsDerivatives<Nx,Nu>
    dynamics(const VectorX x, const VectorU u, const double dt, const ReturnType type)
    {
        // simple euler integration
        DynamicsDerivatives<Nx,Nu> dyn;
        if(isRearWheel_)
        {
            dyn.f = x +dt*symbolic_functions::f(x,u);
            if (type == WITHOUT_DERIVATIVES)
                return dyn;
            // fx and fu should be based on discretized f!!!
            dyn.fx = MatrixXX::Identity()+ dt*symbolic_functions::dfdx(x,u);
            dyn.fu = dt*symbolic_functions::dfdu(x,u);
            return dyn;
        }
        else
        {
            dyn.f = x +dt*symbolic_functions::f_front(x,u);
            if (type == WITHOUT_DERIVATIVES)
                return dyn;
            // fx and fu should be based on discretized f!!!
            dyn.fx = MatrixXX::Identity()+ dt*symbolic_functions::dfdx_front(x,u);
            dyn.fu = dt*symbolic_functions::dfdu_front(x,u);
            return dyn;
        }


    }

    CostDerivatives<Nx,Nu>
    cost(const VectorX x, const VectorU u, const int idx, const ReturnType type)
    {
        if (!std::isnan(u(0))) {
            // running cost must be computed here
            CostDerivatives<Nx, Nu> obj;
            if(idx>5)
            {
                //obj.c = symbolic_functions::cost(x, u, state_weight_, input_weight_, x_ref[idx]);
                obj.c = symbolic_functions::cost(x, u, state_weight_.array()*std::pow(1.1,1+idx/N),input_weight_, x_ref[idx]);
                if (type == WITHOUT_DERIVATIVES)
                    return obj;
                //obj.cx = symbolic_functions::costx(x, u, state_weight_, input_weight_, x_ref[idx]);
                obj.cx = symbolic_functions::costx(x, u, state_weight_.array()*std::pow(1.1,1+idx/N), input_weight_, x_ref[idx]);

                obj.cu = symbolic_functions::costu(x, u, state_weight_, input_weight_, x_ref[idx]);
                obj.cxx = symbolic_functions::costxx(x, u, state_weight_.array()*std::pow(1.1,1+idx/N), input_weight_, x_ref[idx]);
                obj.cxu = MatrixXU::Zero();
                obj.cuu = symbolic_functions::costuu(x, u, state_weight_, input_weight_, x_ref[idx]);
                return obj;
            }
            else
            {
                obj.c = symbolic_functions::cost(x, u, state_weight_.array()*std::pow(0.01,1-idx/N),input_weight_, x_ref[idx]);
                if (type == WITHOUT_DERIVATIVES)
                    return obj;
                //obj.cx = symbolic_functions::costx(x, u, state_weight_, input_weight_, x_ref[idx]);
                obj.cx = symbolic_functions::costx(x, u, state_weight_.array()*std::pow(0.01,1-idx/N), input_weight_, x_ref[idx]);

                obj.cu = symbolic_functions::costu(x, u, state_weight_, input_weight_, x_ref[idx]);
                obj.cxx = symbolic_functions::costxx(x, u, state_weight_.array()*std::pow(0.01,1-idx/N), input_weight_, x_ref[idx]);
                obj.cxu = MatrixXU::Zero();
                obj.cuu = symbolic_functions::costuu(x, u, state_weight_, input_weight_, x_ref[idx]);
                return obj;
            }



        }
        else
        {
            // final cost must be computed here
            Matrix<double, Nu, 1> u_Zero = Matrix<double, Nu, 1>::Zero();
            CostDerivatives<Nx, Nu> obj;
            obj.c = symbolic_functions::cost(x, u_Zero, final_weight_, input_weight_, x_ref[idx-1]);
            if (type == WITHOUT_DERIVATIVES)
                return obj;
            obj.cx = symbolic_functions::costx(x, u_Zero, final_weight_, input_weight_, x_ref[idx-1]);
            obj.cu = VectorU::Zero();
            obj.cxx = symbolic_functions::costxx(x, u_Zero, final_weight_, input_weight_, x_ref[idx-1]);
            obj.cxu = MatrixXU::Zero();
            obj.cuu = MatrixUU::Zero();
            return obj;
        }
    }

    
    ConstraintDerivatives<Nx,Nu>
    constraint(const VectorX x, const VectorU u, const int idx, const ReturnType type) {
        Matrix<double, 6, 1> x_ = x.block<6, 1>(0, 0);
        Matrix<double, 2, 1> u_ = u.block<2, 1>(0, 0);
        Matrix<double, 4, 1> sfc_modified_temp;
        sfc_modified_temp << sfc_modified[idx].xl,sfc_modified[idx].xu,
                            sfc_modified[idx].yl,sfc_modified[idx].yu; //xl, xu, yl, yu

        int N_obs = obs_q.size();
        if(N_obs==0)
        {
            if(!std::isnan(u_(0)))
            {
                // Stage Constraints
                ConstraintDerivatives<Nx,Nu> obj(Nc,0);
                if (sfc_idx[idx])
                {
                    obj.con = symbolic_functions::con(x_,u_,sfc_modified_temp);
                }
                else
                {
                    obj.con = symbolic_functions::con_exp(x_,u_,sfc_modified_temp);
                }


                if(type == WITHOUT_DERIVATIVES)
                    return obj;
                obj.conx = symbolic_functions::conx(x_,u_,sfc_modified_temp);
                obj.conu = symbolic_functions::conu(x_,u_,sfc_modified_temp);
                return obj;

            }
            else
            {
                ConstraintDerivatives<Nx,Nu> obj(Nc-4,0);
                obj.con = symbolic_functions::con_final(x_,sfc_modified_temp);
                if(type == WITHOUT_DERIVATIVES)
                    return obj;
                obj.conx = symbolic_functions::conx_final(x_,sfc_modified_temp);
                Matrix<double,Nc-4,Nu> temp_zero;
                temp_zero.setZero();
                obj.conu = temp_zero;

                return obj;
            }
        }
        else
        {
            Matrix<double,2,2> Q1;
            Q1 = car_shape;
            double sqrtTrQ1 = sqrt(Q1.trace());

            if(!std::isnan(u_(0)))
            {
                Matrix<double,2,1> q1;
                q1<<x_(0,0),x_(1,0);

                ConstraintDerivatives<Nx,Nu> obj(Nc+N_obs,0);
                for(int i = 0; i<N_obs;i++)
                {
                    Matrix<double,2,1> q2 = obs_q[i][idx];
                    Matrix<double,2,2> Q2 = obs_Q[i][idx];
                    double sqrtTrQ2 = sqrt(Q2.trace());

                    Matrix<double,2,2>  Qsum = (1.0+sqrtTrQ2/sqrtTrQ1)*Q1+(1.0+sqrtTrQ1/sqrtTrQ2)*Q2;
                    Matrix<double,2,2> invQsum = Qsum.inverse();
                    obj.con(Nc+i) = (q1-q2).dot(invQsum*(q1-q2))-1.0;
                    if(type!= WITHOUT_DERIVATIVES)
                    {
                        obj.conx.row(Nc+i).block<1,2>(0,0) = 2.0*(invQsum*(q1-q2)).transpose();
                    }
                }
                // Stage Constraints
                if (sfc_idx[idx])
                {
                    obj.con = symbolic_functions::con(x_,u_,sfc_modified_temp);
                }
                else
                {
                    obj.con = symbolic_functions::con_exp(x_,u_,sfc_modified_temp);
                }
                if(type != WITHOUT_DERIVATIVES)
                {
                    obj.conx.block<Nc,Nx>(0,0) = symbolic_functions::conx(x_,u_,sfc_modified_temp);
                    obj.conu.block<Nc,Nu>(0,0) = symbolic_functions::conu(x_,u_,sfc_modified_temp);
                }
                return obj;
            }
            else
            {
                ConstraintDerivatives<Nx,Nu> obj(Nc-4,0);
                obj.con.block<Nc-4,1>(0,0) = symbolic_functions::con_final(x_,sfc_modified_temp);
                if(type == WITHOUT_DERIVATIVES)
                    return obj;
                obj.conx.block<Nc-4,Nx>(0,0) = symbolic_functions::conx_final(x_,sfc_modified_temp);
                Matrix<double,Nc-4,Nu> temp_zero;
                temp_zero.setZero();
                obj.conu = temp_zero;
                return obj;
            }
        }
    }
};

#endif
