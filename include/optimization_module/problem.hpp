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
/* YW added*/
template<typename T, const int size>
using Collection = std::array<T, size>;

class Problem : public ProblemDescription<Nx,Nu>
{
private:

    Matrix<double,2,1> x_goal_; // Goal : the last point of last corridor
    Collection<Matrix<double,2,1>,50> x_ref; // Reference Tracking Mode
    int isRefUsed;
    Collection<Matrix<double,2,2>,51>& Qx; // shape matrix, constant in plain_MPC

    VectorX final_weight_ = VectorX::Zero(); //Final Cost Weight Factor
    VectorX state_weight_ = VectorX::Zero(); //Running Cost State Weight Factor
    VectorU input_weight_ = VectorU::Zero(); //Running Cost Input Weight Factor

    vector<vector<Matrix2d>>& obs_Q_;
    vector<vector<Vector2d>>& obs_q_;
    Collection<Planner::Corridor,N+1>& sfc_modified;
    bool noConstraint_ = true;

public:
    Problem (Collection<Matrix<double,2,2>,51>& car_shape,
             Collection<Planner::Corridor,N+1>& corridor_seq, vector<vector<Matrix2d>>& obs_Q,
             vector<vector<Vector2d>>& obs_q)
            : sfc_modified(corridor_seq), Qx(car_shape),  obs_Q_(obs_Q), obs_q_(obs_q)
    { }
    void set_goal( const Matrix<double,2,1> x_goal)
    { x_goal_ = x_goal; }

    void set_input_weight( const VectorU weight )
    { input_weight_ = weight;	}

    void set_state_weight( const VectorX weight )
    { state_weight_ = weight; }

    void set_final_weight( const VectorX weight )
    { final_weight_ = weight;	}

    void set_ref(const Collection<Matrix<double,2,1>,50> x_ref_)
    {x_ref = x_ref_;}

    void set_refUsed(const int isRefUsed_)
    {isRefUsed = isRefUsed_;}

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
        if(isRefUsed)
        {
            if(obs_q_.size()>0) {
                if (!std::isnan(u(0))) {
                    // running cost must be computed here
                    CostDerivatives<Nx, Nu> obj;
                    obj.c = symbolic_functions::cost(x, u, state_weight_, input_weight_, x_ref[idx], obs_q_[0][idx]);
                    if (type == WITHOUT_DERIVATIVES)
                        return obj;
                    obj.cx = symbolic_functions::costx(x, u, state_weight_, input_weight_, x_ref[idx], obs_q_[0][idx]);
                    obj.cu = symbolic_functions::costu(x, u, state_weight_, input_weight_, x_ref[idx], obs_q_[0][idx]);
                    obj.cxx = symbolic_functions::costxx(x, u, state_weight_, input_weight_, x_ref[idx], obs_q_[0][idx]);
                    obj.cxu = MatrixXU::Zero();
                    obj.cuu = symbolic_functions::costuu(x, u, state_weight_, input_weight_, x_ref[idx], obs_q_[0][idx]);
                    return obj;
                }
                else
                {
                    // final cost must be computed here
                    Matrix<double, Nu, 1> u_Zero = Matrix<double, Nu, 1>::Zero();
                    CostDerivatives<Nx, Nu> obj;
                    obj.c = symbolic_functions::cost(x, u_Zero, final_weight_, input_weight_, x_ref[idx-1], obs_q_[0][idx]);
                    if (type == WITHOUT_DERIVATIVES)
                        return obj;
                    obj.cx = symbolic_functions::costx(x, u_Zero, final_weight_, input_weight_, x_ref[idx-1], obs_q_[0][idx]);
                    obj.cu = VectorU::Zero();
                    obj.cxx = symbolic_functions::costxx(x, u_Zero, final_weight_, input_weight_, x_ref[idx-1], obs_q_[0][idx]);
                    obj.cxu = MatrixXU::Zero();
                    obj.cuu = MatrixUU::Zero();
                    return obj;
                }
            }
            else
            {
                Matrix<double,2,1> temp_obstacle;
                temp_obstacle<<10000, 20000;
                if (!std::isnan(u(0))) {
                    // running cost must be computed here
                    CostDerivatives<Nx, Nu> obj;
                    obj.c = symbolic_functions::cost(x, u, state_weight_, input_weight_, x_goal_, temp_obstacle);
                    if (type == WITHOUT_DERIVATIVES)
                        return obj;
                    obj.cx = symbolic_functions::costx(x, u, state_weight_, input_weight_, x_goal_, temp_obstacle);
                    obj.cu = symbolic_functions::costu(x, u, state_weight_, input_weight_, x_goal_, temp_obstacle);
                    obj.cxx = symbolic_functions::costxx(x, u, state_weight_, input_weight_, x_goal_, temp_obstacle);
                    obj.cxu = MatrixXU::Zero();
                    obj.cuu = symbolic_functions::costuu(x, u, state_weight_, input_weight_, x_goal_, temp_obstacle);
                    return obj;
                }
                else {
                    // final cost must be computed here
                    Matrix<double, Nu, 1> u_Zero = Matrix<double, Nu, 1>::Zero();
                    CostDerivatives<Nx, Nu> obj;
                    obj.c = symbolic_functions::cost(x, u_Zero, final_weight_, input_weight_, x_goal_, temp_obstacle);
                    if (type == WITHOUT_DERIVATIVES)
                        return obj;
                    obj.cx = symbolic_functions::costx(x, u_Zero, final_weight_, input_weight_, x_goal_, temp_obstacle);
                    obj.cu = VectorU::Zero();
                    obj.cxx = symbolic_functions::costxx(x, u_Zero, final_weight_, input_weight_, x_goal_, temp_obstacle);
                    obj.cxu = MatrixXU::Zero();
                    obj.cuu = MatrixUU::Zero();
                    return obj;
                }
            }
        }
        else
        {
            if(obs_q_.size()>0) {
                if (!std::isnan(u(0))) {
                    // running cost must be computed here
                    CostDerivatives<Nx, Nu> obj;
                    obj.c = symbolic_functions::cost(x, u, state_weight_, input_weight_, x_goal_, obs_q_[0][idx]);
                    if (type == WITHOUT_DERIVATIVES)
                        return obj;
                    obj.cx = symbolic_functions::costx(x, u, state_weight_, input_weight_, x_goal_, obs_q_[0][idx]);
                    obj.cu = symbolic_functions::costu(x, u, state_weight_, input_weight_, x_goal_, obs_q_[0][idx]);
                    obj.cxx = symbolic_functions::costxx(x, u, state_weight_, input_weight_, x_goal_, obs_q_[0][idx]);
                    obj.cxu = MatrixXU::Zero();
                    obj.cuu = symbolic_functions::costuu(x, u, state_weight_, input_weight_, x_goal_, obs_q_[0][idx]);
                    return obj;
                }
                else
                {
                    // final cost must be computed here
                    Matrix<double, Nu, 1> u_Zero = Matrix<double, Nu, 1>::Zero();
                    CostDerivatives<Nx, Nu> obj;
                    obj.c = symbolic_functions::cost(x, u_Zero, final_weight_, input_weight_, x_goal_, obs_q_[0][idx]);
                    if (type == WITHOUT_DERIVATIVES)
                        return obj;
                    obj.cx = symbolic_functions::costx(x, u_Zero, final_weight_, input_weight_, x_goal_, obs_q_[0][idx]);
                    obj.cu = VectorU::Zero();
                    obj.cxx = symbolic_functions::costxx(x, u_Zero, final_weight_, input_weight_, x_goal_, obs_q_[0][idx]);
                    obj.cxu = MatrixXU::Zero();
                    obj.cuu = MatrixUU::Zero();
                    return obj;
                }
            }
            else
            {
                Matrix<double,2,1> temp_obstacle;
                temp_obstacle<<10000, 20000;
                if (!std::isnan(u(0))) {
                    // running cost must be computed here
                    CostDerivatives<Nx, Nu> obj;
                    obj.c = symbolic_functions::cost(x, u, state_weight_, input_weight_, x_goal_, temp_obstacle);
                    if (type == WITHOUT_DERIVATIVES)
                        return obj;
                    obj.cx = symbolic_functions::costx(x, u, state_weight_, input_weight_, x_goal_, temp_obstacle);
                    obj.cu = symbolic_functions::costu(x, u, state_weight_, input_weight_, x_goal_, temp_obstacle);
                    obj.cxx = symbolic_functions::costxx(x, u, state_weight_, input_weight_, x_goal_, temp_obstacle);
                    obj.cxu = MatrixXU::Zero();
                    obj.cuu = symbolic_functions::costuu(x, u, state_weight_, input_weight_, x_goal_, temp_obstacle);
                    return obj;
                }
                else {
                    // final cost must be computed here
                    Matrix<double, Nu, 1> u_Zero = Matrix<double, Nu, 1>::Zero();
                    CostDerivatives<Nx, Nu> obj;
                    obj.c = symbolic_functions::cost(x, u_Zero, final_weight_, input_weight_, x_goal_, temp_obstacle);
                    if (type == WITHOUT_DERIVATIVES)
                        return obj;
                    obj.cx = symbolic_functions::costx(x, u_Zero, final_weight_, input_weight_, x_goal_, temp_obstacle);
                    obj.cu = VectorU::Zero();
                    obj.cxx = symbolic_functions::costxx(x, u_Zero, final_weight_, input_weight_, x_goal_, temp_obstacle);
                    obj.cxu = MatrixXU::Zero();
                    obj.cuu = MatrixUU::Zero();
                    return obj;
                }
            }
        }

    }
    ConstraintDerivatives<Nx,Nu>
    constraint(const VectorX x, const VectorU u, const int idx, const ReturnType type) {
//        int N_obs = obs_q_.size();
        Matrix<double, 5, 1> x_ = x.block<5, 1>(0, 0);
        Matrix<double, 2, 1> u_ = u.block<2, 1>(0, 0);
        Matrix<double, 4, 1> sfc_modified_temp;
        sfc_modified_temp << sfc_modified[idx].xl,sfc_modified[idx].xu,
                            sfc_modified[idx].yl,sfc_modified[idx].yu; //xl, xu, yl, yu
        if(!std::isnan(u_(0)))
        {

            ConstraintDerivatives<Nx,Nu> obj(8,0);
            obj.con = symbolic_functions::con(x_,u_,sfc_modified_temp);
            if(type == WITHOUT_DERIVATIVES)
                return obj;
            obj.conx = symbolic_functions::conx(x_,u_,sfc_modified_temp);
            obj.conu = symbolic_functions::conu(x_,u_,sfc_modified_temp);
            return obj;

        }
        else
        {
            Matrix<double,2,1> u_null;
            ConstraintDerivatives<Nx,Nu> obj(8,0);
            obj.con = symbolic_functions::con_final(x_,u_null,sfc_modified_temp);
            if(type == WITHOUT_DERIVATIVES)
                return obj;
            obj.conx = symbolic_functions::conx_final(x_,u_null,sfc_modified_temp);
            obj.conu = MatrixXU::Zero();
            return obj;
        }
    }
};

#endif