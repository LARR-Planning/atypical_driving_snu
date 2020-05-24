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
    ilqr_param.tolGrads = power(10.0, VectorXd::LinSpaced(4, -4.0, -6.0));
    ilqr_param.tolCosts = power(10.0, VectorXd::LinSpaced(4, -2.0, -6.0));
    ilqr_param.tolConsts = power(10.0, VectorXd::LinSpaced(4, -2.0, -6.0));
    ilqr_param.alphas = power(10.0, VectorXd::LinSpaced(5, 0.0, -3.0));
    ilqr_param.maxIter = 1000;
    ilqr_param.mu = 3.0;
    ilqr_param.lambda = 0.0;
    ilqr_param.phi = 0.1;
    ilqr_param.verbosity = 0;
    ilqr_param.dmu = 1.2;
    ilqr_param.dphi = 0.8;

    carDefaultShape << 4.0, 0.0,
                        0.0, 4.0;
    for(int i = 0; i<51;i++)
    {
        bodyArray[i]<< 4.0, 0.0, 0.0, 4.0;
    }
    state_weight_<< 0.5, 0.5, 0.5 , 0.05 , 0.05;
    final_weight_<< 0.5, 0.5, 0.5 , 0.05 , 0.05;
    input_weight_<< 0.2,1.0;
    isRefUsed = 0;
    cout << "[LocalPlanner] Init." << endl;
    
}
/**
 * Update the planning result to p_base
 */
void LocalPlanner::updateTrajToBase(){
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
    //vector<Vector2d,Eigen::aligned_allocator<Vector2d>> path_temp;
    //vector<Matrix2d,Eigen::aligned_allocator<Matrix2d>> shape_temp;
    vector<Matrix2d> shape_temp;
    vector<Vector2d> path_temp;
    shape_temp.clear();
    path_temp.clear();
    //Collection<Matrix<double,2,2>,51> shape_temp;
//    obs_q.resize(0);
//    obs_Q.resize(0);
    if (obs_q.size())
        obs_q.clear();
    if (obs_Q.size())
        obs_Q.clear();
//    obs_Q.resize(1);
//    obs_q.resize(1);
//    cout << "Size of obstacle path array " <<p_base->getCurObstaclePathArray().obstPathArray.size()<< endl;

    if(p_base->getCurObstaclePathArray().obstPathArray.size()>0)
    {
        int count_id = 0;
        for (auto s : p_base->getCurObstaclePathArray().obstPathArray) {
            for (int i = 0; i < 50; i++)
            {
                path_temp.push_back(s.obstPath[i].q);
                shape_temp.push_back(s.obstPath[i].Q.inverse());
//                obs_Q[count_id][i] = s.obstPath[i].Q.inverse();
//                obs_q[count_id][i] = s.obstPath[i].q;
//                if (shape_temp[i].rows() != 2 or shape_temp[i].cols() != 2)
//                    cout << "shape_temp size invalid" <<endl;
            }
            path_temp.push_back(s.obstPath[49].q);
            shape_temp.push_back(s.obstPath[49].Q.inverse());
            obs_Q.push_back(shape_temp);
            obs_q.push_back(path_temp);
//           shape_temp.clear();
//           path_temp.clear();
            count_id++;
        }
        //cout << "loop size " <<count_id<< endl;
    }
}

Matrix<double,2,1> LocalPlanner::getLocalGoal(){
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
    Matrix<double,2,1> tempLocalGoal;
    tempLocalGoal<< p_base->getSkeletonPath().at(goal_index).x, p_base->getSkeletonPath().at(goal_index).y;
    return tempLocalGoal;
}

void LocalPlanner::SfcToOptConstraint(double t){
    double t_end_;
    double t_start_  = 0.0;
    int N_corr = 0;
    int count1 = 0;
    int count2 = 0;
    int count3 = 1;
    for(auto &s: p_base->getCorridorSeq(t,t+param.horizon)) {
        if (N_corr < 51) {
            if (count1 * count2 == 0) {
                box_constraint[count2] = s;
                count1++;
                count2++;
                N_corr++;
            }
            t_end_ = s.t_end;
            if (t_end_ > param.horizon) {
                for (int i = 0; i < round((param.horizon- t_start_)/param.tStep); i++) {
                    box_constraint[count2] = s;
                    count2++;
                    N_corr++;
                }
            }
            else {

                for (int i = 0; i < round((t_end_ - t_start_) / param.tStep); i++) {
                    box_constraint[count2] = s;
                    count2++;
                    N_corr++;
                }
                t_start_ = param.tStep * (count2 - 1);
            }
//            cout<<count3<< " th Corridor"<<endl;
//            cout<< "xl: "<< s.xl<< "[m] xu: "<< s.xu<< "[m] yl: "<< s.yl<< "[m] yl: "<< s.yu<<"[m]"<<endl;
//            cout <<"t_start: "<<s.t_start <<"[s] t_end: "<<s.t_end<<"[s]"<<endl;
//            count3++;
        }
    }
}
 Collection<Corridor,51> LocalPlanner::getOptCorridor()
 {
    return box_constraint;
 }

 void LocalPlanner::SetLocalWpts()
 {
    Matrix<double,2,1> wpts_temp;
    Matrix<double,2,1> wpts_temp1;
    vector<Matrix<double,2,1>> node_list;
    vector<int> wptsNumber_list;
    vector<double> t_list;
    for (auto& ss : p_base->getLanePath().lanes)
    {
        for(auto &tt: ss.laneCenters)
        {
            wpts_temp<<tt.x, tt.y;
            node_list.push_back(wpts_temp);
        }
    }


    double node_distance = 0.0;
    int number_temp = 0;
    double t_temp = 0;
    double t_start_ = 0;
    wpts_temp = node_list[0];
    int count = 0;
    for(int i = 1; i<node_list.size();i++)
    {
        t_list.push_back(t_temp);
        node_distance = (node_list[i]-node_list[i-1]).norm();
       t_temp +=node_distance * 0.5; // divided by 2m/s --> wpts list generation at Global Planner.cpp is opimal;

    }
     Matrix<double,2,1> velNormalized_temp;
    Matrix<double,3,1> wpts_temp2;
     vector<Matrix<double,2,1>> wpts_list1;
     vector<Matrix<double,3,1>> wpts_list2;

    for (int i = 0; i< t_list.size();i++)
    {
        if(i == 0)
        {
            velNormalized_temp = (node_list[1]-node_list[0]).normalized();
            wpts_temp1=node_list[0];
            wpts_temp2(0)=wpts_temp1(0);
            wpts_temp2(1)=wpts_temp1(1);
            wpts_temp2(2)=atan2(velNormalized_temp(1),velNormalized_temp(0));
            count++;
        }
        else
        {
            for (int j = 0; j < round((t_list[i] - t_start_) / param.tStep); j++) {
                velNormalized_temp = (node_list[i]-node_list[i-1]).normalized();
                wpts_temp1 = node_list[i-1] + (t_start_+(j+1)*dt - t_list[i-1])*velNormalized_temp*2; //multiplied by 2m/s
                wpts_temp2(0) = wpts_temp1(0);
                wpts_temp2(1) =wpts_temp1(1);
                wpts_temp2(2) =atan2(velNormalized_temp(1),velNormalized_temp(0));
                wpts_list1.push_back(wpts_temp1);
                wpts_list2.push_back(wpts_temp2);
                count++;
            }
        }
        t_start_ = param.tStep * (count- 1);
    }

    Matrix<double,2,1> curr_pos;
    curr_pos<< p_base->getCarState().x, p_base->getCarState().y;
    int start_idx = 0;
    double min_gap = 10000;
    for(int i = 0; i<wpts_list1.size();i++)
    {
        if(min_gap>(curr_pos-wpts_list1[i]).norm())
        {
            min_gap = (curr_pos-wpts_list1[i]).norm();
            start_idx = i;
        }

    }
    Matrix<double,2,1> direction_my;
    Matrix<double,2,1> direction_path;
    direction_my<< cos(p_base->getCarState().theta), sin(p_base->getCarState().theta);
    direction_path<< p_base->getCarState().x-wpts_list2[start_idx].coeffRef(0,0),
            p_base->getCarState().y-wpts_list2[start_idx].coeffRef(1,0);
//    double flag_changeIdx = (direction_my.array()*direction_path.array()).sum();
//    while(flag_changeIdx<0)
//    {
//        flag_changeIdx = (direction_my.array()*direction_path.array()).sum();
//        start_idx+=1;
//    }
    for(int i = 0; i<50; i++)
    {
        if(start_idx+i>wpts_list2.size())
        {
            local_wpts[i] = wpts_list2[wpts_list2.size()];
        }
        else
        {
            local_wpts[i] = wpts_list2[start_idx+i];
        }
    }

//
//    p_base->getLanePath().lanes[0].laneCenters[0].x;
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
    cout << "[LocalPlanner] Initialized.. " << endl;
//    cout << "[LocalPlanner] Done. " << endl;
//    cout<< "I am in the Local Planner plan function"<<endl;
     cout<<"---------------------------------"<<endl;
     Matrix<double,2,1> x_goal_;
     x_goal_ = LocalPlanner::getLocalGoal();
     cout<< "New Local Goal is"<<x_goal_.coeffRef(0,0)<<"and"<<x_goal_.coeffRef(1,0)<<endl;
     cout<<"Current car Speed: "<<p_base->getCarState().v<<" [m/s]"<<endl;
     cout<<"Current x-position: "<<p_base->getCarState().x<< " [m]"<<endl;
     cout<<"Current y-position: "<<p_base->getCarState().y<< " [m]"<<endl;
     cout<<"Current heading angle: "<<p_base->getCarState().theta*180/3.1415926535<< " [deg]"<<endl;

     LocalPlanner::SfcToOptConstraint(t); // convert SFC to box constraints

     LocalPlanner::ObstToConstraint();
     if(p_base->getLanePath().lanes.size()>0)
     {
         LocalPlanner::SetLocalWpts();
         state_weight_.coeffRef(0,0) = 0.5; //x
         state_weight_.coeffRef(1,0) = 0.5; //y
         state_weight_.coeffRef(2,0) = 0.5; //v
         state_weight_.coeffRef(3,0) = 0.1; //delta
         state_weight_.coeffRef(4,0) = 0.01; //theta

         final_weight_.coeffRef(0,0) = 0.5; // x
         final_weight_.coeffRef(1,0) = 0.5; //y
         final_weight_.coeffRef(2,0) = 0.5; //v
         final_weight_.coeffRef(3,0) = 0.1; //delta
         final_weight_.coeffRef(4,0) = 0.01; // theta

         input_weight_.coeffRef(0,0) = 0.05;
         input_weight_.coeffRef(1,0) = 0.01;
         isRefUsed = 1;
     }

    // LocalPlanner::SetLocalWpts();
     using namespace Eigen;

     //Following codes will be wrapped with another wrapper;
     std::shared_ptr<Problem> prob = std::make_shared<Problem>(bodyArray, box_constraint, obs_Q, obs_q);
     // Do not have to be defined every loop

     bool noConstraint_ = 0;
     prob->set_state_weight(state_weight_);
     prob->set_final_weight(final_weight_);
     prob->set_input_weight(input_weight_);
     prob->set_noConstraint(noConstraint_);
     prob->set_refUsed(isRefUsed);
     prob->set_goal(x_goal_);
     prob->set_ref(local_wpts);


     static int loop_num = 0;
     std::array<Matrix<double,Nu,1>,N> u0;
     Matrix<double,Nx,1> x0_new;
     Collection<Matrix<double,Nu,1>,N> uN_new;
     Collection<Matrix<double,Nx,1>,N+1> xN_new;
     // if car state contains strange value;
     if(isnan(p_base->getCarState().theta) || isnan(p_base->getCarState().theta)|| (p_base->getCarState().v>1000)) {
         for (int i = 0; i < N; i++) {
             xN_new[i].setZero();
             uN_new[i].setZero();
         }
         xN_new[N].setZero();
     }
     else{
         if(loop_num == 0)
         {
             for(auto &s :u0)
             {
                 s=(Matrix<double,Nu,1>()<< 1,0.0).finished();
             }
             x0_new = (Matrix<double,Nx,1>()<<p_base->getCarState().x, p_base->getCarState().y,p_base->getCarState().v,
                     0.0, p_base->getCarState().theta).finished();
             cout<<"current x: " <<p_base->getCarState().x<<endl;
             cout<<"current y: " <<p_base->getCarState().y<<endl;
             cout<<"current v: " <<p_base->getCarState().v<<endl;
             cout<<"current theta: " <<p_base->getCarState().theta*180/3.1415926535<<"[deg]"<<endl;
             uN_new = u0;
             iLQR<Nx,Nu,N> ilqr_init(*prob, x0_new,uN_new,dt,ilqr_param);
             ilqr_init.solve();


             uN_NextInit = ilqr_init.uN_;
             uN_new = ilqr_init.uN_;
             xN_new = ilqr_init.xN_;
//            uN_NextInit =uN_new;
             for(int j = 0;j<49;j++)
             {
                 uN_NextInit[j] = uN_new[j+1];
             }
             uN_NextInit[49]= uN_new[49];
//             next_state = xN_new[1];

//             cout<<"-------"<<endl;
//             for(int j = 0; j<50;j++)
//             {
//
//                 cout<<uN_new[j].coeff(0,0)<<endl;
//             }
//             cout<<"So far, new input, from now on, new states"<<endl;
//             for(int j = 0; j<51;j++)
//             {
//                 cout<<"updated new future x-position: "<<xN_new[j].coeff(0,0)<<endl;
//             }
             loop_num++;
         }
         else
         {
             //  Be executed after initial Loop (loop_num>0)
             x0_new = (Matrix<double,Nx,1>()<<p_base->getCarState().x, p_base->getCarState().y,p_base->getCarState().v,
                     next_state(3,0), p_base->getCarState().theta).finished();
//             x0_new = (Matrix<double,Nx,1>()<<next_state(0,0), next_state(1,0),next_state(2,0),
//                     next_state(3,0), next_state(4,0)).finished();

             //             x0_new = (Matrix<double,Nx,1>()<<p_base->getCarState().x, p_base->getCarState().y,p_base->getCarState().v,
//                     p_base->getCurInput(t).steer_angle_cmd, p_base->getCarState().theta).finished();
             iLQR<Nx,Nu,N> ilqr(*prob, x0_new,uN_NextInit,dt,ilqr_param);
             ilqr.solve();
             uN_new = ilqr.uN_;
             xN_new = ilqr.xN_;
             next_state = xN_new[1];
//             cout<<"-------"<<endl;
//             for(int j = 0; j<50;j++)
//             {
//
//                 cout<<"Accleration of Car"<<uN_new[j].coeff(0,0)<<endl;
//             }
//             for(int j = 0; j<51;j++)
//             {
//                 cout<<"updated new future x-position: "<<xN_new[j].coeff(0,0)<<" [m]"<<endl;
//             }
//             for(int j = 0; j<51;j++)
//             {
//                 cout<<"updated new future steering angle "<<xN_new[j].coeff(3,0)*180/3.1415926535<<" [deg]"<<endl;
//             }
//             uN_NextInit =uN_new;
             for(int j = 0;j<49;j++)
             {
                 uN_NextInit[j] = uN_new[j+1];
             }
             uN_NextInit[49]= uN_new[49];
         }
     }

     // Update CarState and CarInput into the form designed in PlannerCore header file
     Matrix<double,50,1> ts_temp = VectorXd::LinSpaced(50,0.0,4.9);
     ts_temp = ts_temp.array()+ t;
     CarState carState_temp;
     CarInput carInput_temp;

     curPlanning.ts.clear();
     curPlanning.xs.clear();
     curPlanning.us.clear();
     for(int i = 0 ;i<50; i++)
     {
         carState_temp.x = xN_new[i].coeffRef(0,0);
         carState_temp.y = xN_new[i].coeffRef(1,0);
         carState_temp.v = xN_new[i].coeffRef(2,0);
         carState_temp.theta = xN_new[i].coeffRef(4,0);

         carInput_temp.alpha = uN_new[i].coeffRef(0,0);
         carInput_temp.delta = xN_new[i].coeffRef(3,0);

         curPlanning.ts.push_back(ts_temp.coeffRef(i,0));
         curPlanning.xs.push_back(carState_temp);
         curPlanning.us.push_back(carInput_temp);
     }
    //loop_num++;
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








