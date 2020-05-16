 //
// Created by jbs on 20. 4. 11..
//

#include <atypical_planner/LocalPlanner.h>
#include <optimization_module/problem.hpp>
#include <optimization_module/dimension.h>
#include <math.h>
#include <cmath>
using namespace Planner;

///////////////////////////
//   Local Planner base  //
///////////////////////////


/**
 * @brief Construction of local planner
 * @param l_param local planning paramters
 * @param p_base_ Planning base (input + output)
 * @param m mutex set
 */
LocalPlanner::LocalPlanner(const Planner::ParamLocal &l_param,
                           shared_ptr<PlannerBase> p_base_): AbstractPlanner(p_base_),param(l_param){
    ilqr_param.rho = 1e-5;
    ilqr_param.drho = 1.0;
    ilqr_param.rhoFactor = 1.6;
    ilqr_param.rhoMax = 1e10;
    ilqr_param.rhoMin = 1e-6;
    ilqr_param.tolGrads = power(10.0, VectorXd::LinSpaced(30, -4.0, -6.0));
    ilqr_param.tolCosts = power(10.0, VectorXd::LinSpaced(30, -2.0, -6.0));
    ilqr_param.tolConsts = power(10.0, VectorXd::LinSpaced(30, -2.0, -6.0));
    ilqr_param.alphas = power(10.0, VectorXd::LinSpaced(5, 0.0, -3.0));
    ilqr_param.maxIter = 1000;
    ilqr_param.mu = 2.0;
    ilqr_param.lambda = 0.0;
    ilqr_param.phi = 1e-4;
    ilqr_param.verbosity = 1;
    ilqr_param.dmu = 2.0;
    ilqr_param.dphi = 0.8;

    carDefaultShape << 4.0, 0.0,
                        0.0, 4.0;
    for(int i = 0; i<51;i++)
    {
        bodyArray[i]<< 4.0, 0.0, 0.0, 4.0;
    }
    state_weight_<< 0.05, 0.05, 0.05, 0.05, 0.05;
    final_weight_<< 0.05, 0.05, 0.05, 0.05, 0.05;
    input_weight_<< 0.01, 0.01;

    cout << "[LocalPlanner] Init." << endl;
    
}
/**
 * Update the planning result to p_base
 */
void LocalPlanner::updateTrajToBase() {
    // update routine here
    p_base->setMPCResultTraj(curPlanning); // just an example
}
/**
 * @brief moniter whether current path is feasible against the obstaclePathArray
 * @return
 */
bool LocalPlanner::isCurTrajFeasible() {

    return true;
}
void LocalPlanner::QxFromPrediction(Collection<double,51> mpcPredictionHeads)
{
    Matrix<double,2,2> rotationMatrix;
    for (int i = 0 ; i <51; i++)
    {
        double theta = mpcPredictionHeads[i];
        rotationMatrix<< cos(theta), -sin(theta),
                        sin(theta), cos(theta);
        bodyArray[i] = rotationMatrix * carDefaultShape * rotationMatrix.transpose();
    }
}
void LocalPlanner::ObstToConstraint() {
    int count_id = 0;
    Collection<Matrix<double,2,1>,51> path_temp;
    Collection<Matrix<double,2,2>,51> shape_temp;
    if(p_base->getCurObstaclePathArray().obstPathArray.size()>0) {
        for (auto &s : p_base->getCurObstaclePathArray().obstPathArray) {
            for (int i = 0; i < 51; i++) {
                path_temp[i].block<2, 1>(0, 0) = s.obstPath[i].q;
                shape_temp[i].block<2, 2>(0, 0) = s.obstPath[i].Q.inverse();
            }
            obs_q.push_back(path_temp);
            obs_Q.push_back(shape_temp);
            count_id++;
        }
    }
}

Point LocalPlanner::getLocalGoal(){
    double SP_EPSILON = 1e-9;
    int box_index = -1;
    for(int i = 0; i < p_base->getCorridorSeq().size(); i++){
        Corridor corridor = p_base->getCorridorSeq().at(i);
        if(corridor.t_end >= param.horizon){
            box_index = i;
            break;
        }
    }
    Corridor lastBox = p_base->getCorridorSeq().at(box_index);
    int goal_index = -1;
    for(int i = 0; i < p_base->getSkeletonPath().size(); i++){
        Point skeletonPoint = p_base->getSkeletonPath().at(i);
        if(skeletonPoint.x > lastBox.xl - SP_EPSILON
           && skeletonPoint.y > lastBox.yl - SP_EPSILON
           && skeletonPoint.x < lastBox.xu + SP_EPSILON
           && skeletonPoint.y < lastBox.yu + SP_EPSILON)
        {
            goal_index = i;
        }
        else if(goal_index >= 0){
            break;
        }
    }
    return p_base->getSkeletonPath().at(goal_index);
}

void LocalPlanner::SfcToOptConstraint(){
    double t_end_;
    double t_start_  = 0.0;
    int N_corr = 0;
    int count1 = 0;
    int count2 = 0;

    for(auto &s: p_base->getCorridorSeq()) {
        if (N_corr < 51) {
            if (count1 * count2 == 0) {
                box_constraint[count2] = s;
                count1++;
                count2++;
                N_corr++;
            }

            t_end_ = s.t_end;
            if (t_end_ > param.horizon) {
                for (int i = 0; i < floor((param.horizon - t_start_) / param.tStep); i++) {
                    box_constraint[count2] = s;
                    count2++;
                    N_corr++;
                }
            }
            else {
                for (int i = 0; i < floor((t_end_ - t_start_) / param.tStep); i++) {
                    box_constraint[count2] = s;
                    count2++;
                    N_corr++;
                }
                t_start_ = param.tStep * (count2 - 1);
            }
        }
    }
}
 Collection<Corridor,51> LocalPlanner::getOptCorridor()
 {
    return box_constraint;
 }

 LocalPlannerPlain::LocalPlannerPlain(const Planner::ParamLocal &l_param,
                                      shared_ptr<PlannerBase> p_base_) :LocalPlanner(l_param,p_base_) {
     cout << "[LocalPlanner] Plain MPC mode engaged." << endl;
 }
/**
 * @brief Plan with plainMPC. Every elements are deterministic
 * @return
 */
bool LocalPlannerPlain::plan(double t) {
//    Checked that this function is always executed
//    cout << "[LocalPlanner] planning... " << endl;
//    cout << "[LocalPlanner] Done. " << endl;
//    cout<< "I am in the Local Planner plan function"<<endl;
    LocalPlanner::SfcToOptConstraint(); // convert SFC to box constraints
//    cout<<p_base->getCurObstaclePathArray().obstPathArray[0].obstPath.size()<<endl;
//    cout<<p_base->getCurObstaclePathArray().obstPathArray[0].obstPath.size()<<endl;
    //cout<<p_base->getCurObstaclePathArray().obstPathArray[0].obstPath[0].q(0,0)<<endl;
    LocalPlanner::ObstToConstraint();
    // cout<< LocalPlanner::box_constraint[1].yl<<endl;
    using namespace Eigen;
//    //Following codes will be wrapped with another wrapper;

    std::shared_ptr<Problem> prob = std::make_shared<Problem>(bodyArray, box_constraint, obs_Q, obs_q);
    // Do not have to be defined every loop

     bool noConstraint_ = 0;
     prob->set_state_weight(state_weight_);
     prob->set_final_weight(final_weight_);
     prob->set_input_weight(input_weight_);
     prob->set_noConstraint(noConstraint_);

////     // A new goal-update function will be defined/added with argument x_goal
     Matrix<double,2,1> x_goal_;
     x_goal_<<10.0, 0.0;
     prob->set_goal(x_goal_);
     static int loop_num = 0;

     std::array<Matrix<double,Nu,1>,N> u0;
     if(loop_num == 0)
     {
         for(auto &s :u0)
         {
             s=(Matrix<double,Nu,1>()<< 0.0,0.0).finished();
         }
     }

    Matrix<double,Nx,1> x0_new; //= (Matrix<double,Nx,1>()<<p_base->getCarState().x, p_base->getCarState().y,p_base->getCarState().v,p_base->getCurInput(t).a0ccel_decel_cmd, p_base->getCarState().theta).finished();
    Collection<Matrix<double,Nu,1>,N> uN_new = u0;

    if(loop_num == 0)
    {
        uN_new = u0;
        iLQR<Nx,Nu,N> ilqr_init(*prob, x0_new,uN_new,dt,ilqr_param);
        ilqr_init.solve();
        uN_new = ilqr_init.uN_;
        for(int j = 0; j<50;j++)
        {
            cout<<uN_new[j](0)<<endl;
        }

        loop_num++;
        cout<< "Wow No error??"<<endl;
    }
    else
    {
//        iLQR<Nx,Nu,N>ilqr(*prob,x0_new, uN_new,dt,ilqr_param);
//        ilqr.solve();
//        cout<<"No error, Congratulations"<<endl;
    }


    //TODO: print out the outcome of the planning

    return true; // change this line properly
}







/////////////////////////////////
//   Local Planner-Stochastic  //
/////////////////////////////////


LocalPlannerStochastic::LocalPlannerStochastic(const Planner::ParamLocal &l_param,
                                               shared_ptr<PlannerBase> p_base_) :LocalPlanner(l_param,p_base_) {
    cout << "[LocalPlanner] Stochastic MPC mode engaged." << endl;
}

/**
 * @brief Plan with plainMPC. Every elements are deterministic
 * @return
 */
bool LocalPlannerStochastic::plan(double t ) {

//    cout << "[LocalPlanner] planning... " << endl;

//    cout << "[LocalPlanner] Done. " << endl;

    //TODO: print out the outcome of the planning

    return true; // change this line properly
}








