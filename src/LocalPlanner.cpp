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
    ilqr_param.tolGrads = power(10.0, VectorXd::LinSpaced(5, -4.0, -6.0));
    ilqr_param.tolCosts = power(10.0, VectorXd::LinSpaced(5, -2.0, -6.0));
    ilqr_param.tolConsts = power(10.0, VectorXd::LinSpaced(5, -2.0, -6.0));
    ilqr_param.alphas = power(10.0, VectorXd::LinSpaced(5, 0.0, -3.0));
    ilqr_param.maxIter = 1000;
    ilqr_param.mu = 1.5;
    ilqr_param.lambda = 0.0;
    ilqr_param.phi = 0.1;
    ilqr_param.verbosity = 0;
    ilqr_param.dmu = 1.2;
    ilqr_param.dphi = 0.8;

    car_shape << 2.0, 0.0, 0.0, 2.0;
    state_weight_<< 0.5, 0.5, 0.0, 0.0, 0.0, 0.05;
    final_weight_<< 0.5, 0.5, 0.0, 0.0, 0.0, 1.0;
    input_weight_<< 0.005, 0.5;
    isRefUsed = 0;
    N_corr = 5;
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
/*
void LocalPlanner::QxFromPrediction(Collection<double,N+1> mpcPredictionHeads)
{
    Matrix<double,2,2> rotationMatrix;
    for (int i = 0 ; i <N+1; i++)
    {
        double theta = mpcPredictionHeads[i];
        rotationMatrix<< cos(theta), -sin(theta),
                        sin(theta), cos(theta);
        bodyArray[i] = rotationMatrix * carDefaultShape * rotationMatrix.transpose();
    }
}
*/

void LocalPlanner::ObstToConstraint() {

    vector<Matrix2d> shape_temp;
    vector<Vector2d> path_temp;
//    shape_temp.clear();
//    path_temp.clear();
    //Collection<Matrix<double,2,2>,51> shape_temp;
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
        Vector2d car_pos;
        car_pos<< p_base->getCarState().x, p_base->getCarState().y;
        for (auto s : p_base->getCurObstaclePathArray().obstPathArray) {
            if((s.obstPath[0].q - car_pos).norm()<30)
            {
                for (int i = 0; i < N; i++)
                {
                    path_temp.push_back(s.obstPath[i].q);
                    shape_temp.push_back(s.obstPath[i].Q.inverse());
                }
                path_temp.push_back(s.obstPath[N-1].q);
                shape_temp.push_back(s.obstPath[N-1].Q.inverse());
                obs_Q.push_back(shape_temp);
                obs_q.push_back(path_temp);
            }
            count_id++;
        }
        //cout << "loop size " <<count_id<< endl;
    }
}
/*
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
*/
void LocalPlanner::SfcToOptConstraint(double t){
    //VectorXd time_knots;
    //time_knots.setLinSpaced(N+1,t,t+param.horizon);
    vector<double> time_knots;
    for(int i =0; i<N+1;i++)
    {
        time_knots.push_back(t+i*param.tStep);
    }


    vector<Corridor> queriedCorridors;
     queriedCorridors = p_base->expandCorridors(time_knots, 0.5);

    for(int i = 0; i<N+1;i++)
    {
        box_constraint[i]= queriedCorridors[i];
    }
    // box_csontraint[]


//            cout<<count3<< " th Corridor"<<endl;
//            cout<< "xl: "<< s.xl<< "[m] xu: "<< s.xu<< "[m] yl: "<< s.yl<< "[m] yl: "<< s.yu<<"[m]"<<endl;
//            cout <<"t_start: "<<s.t_start <<"[s] t_end: "<<s.t_end<<"[s]"<<endl;
//            count3++;

     LocalPlanner::SetSfcIdx(N_corr); // choose truly effective SFC

     //TODO: test corridors for debugging delete this after debugging - jungwon
     vector<Corridor> effectiveCorridors;
     for(int i_corr = 0; i_corr < N+1; i_corr++){
         if(sfc_idx[i_corr]){
             effectiveCorridors.emplace_back(queriedCorridors[i_corr]);
         }
     }

     p_base->mSet[1].lock();
     p_base->corridor_seq = effectiveCorridors;
     p_base->mSet[1].unlock();

 }

void LocalPlanner::SetSfcIdx(int N_corr)
{
    int  temp_idx;
    for (int i =0;i<N_corr+1;i++)
    {
        sfc_idx[i] = false;
    }
    sfc_idx[N] =true;

    for(int i =1;i<N_corr;i++)
    {
        temp_idx = N-round(N/N_corr*i);
        sfc_idx[temp_idx] = true;
    }
}

 Collection<Corridor,N+1> LocalPlanner::getOptCorridor()
 {
    return box_constraint;
 }

 void LocalPlanner::SetLocalWpts(double t)
 {
    VectorXd time_knots;
    time_knots.setLinSpaced(N+1,t,t+N*param.tStep);
    vector<Vector2d> pos_ref;
    
    Matrix<double,N+1,1> th_ref;
    // zero th position 
    pos_ref.push_back(p_base->laneSmooth.evalX(p_base->laneSmooth.points,time_knots[0]));
    for (int i = 1; i<N+1;i++)
    {
        pos_ref.push_back(p_base->laneSmooth.evalX(p_base->laneSmooth.points, time_knots[i]));
        th_ref[i-1] =
            atan2((pos_ref[i][1]-pos_ref[i-1][1]),(pos_ref[i][0]-pos_ref[i-1][0]));
    }
    th_ref[N] = th_ref[N-1];


    for (int i = 0;i<N+1;i++)
    {
//        cout<<"At time "<< time_knots[i] <<"[s] pos ref x is: "<<pos_ref[i][0]<<" pos ref y is: "<<pos_ref[i][1]<<endl;
        local_wpts[i][0] = pos_ref[i][0];
        local_wpts[i][1] = pos_ref[i][1];
        local_wpts[i][2] = th_ref[i];
    }
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
//     cout<< "New Local Goal is"<<x_goal_.coeffRef(0,0)<<"and"<<x_goal_.coeffRef(1,0)<<endl;
     cout<<"Current car Speed: "<<p_base->getCarState().v<<" [m/s]"<<endl;
     cout<<"Current x-position: "<<p_base->getCarState().x<< " [m]"<<endl;
     cout<<"Current y-position: "<<p_base->getCarState().y<< " [m]"<<endl;
     cout<<"Current heading angle: "<<p_base->getCarState().theta*180/3.1415926535<< " [deg]"<<endl;
     LocalPlanner::SfcToOptConstraint(t); // convert SFC to box constraints

     cout << "[LocalPlanner] finished constraint conversion.. " << endl;
     LocalPlanner::SetLocalWpts(t);

     cout << "[LocalPlanner] finished ref traj.. " << endl;
     LocalPlanner::ObstToConstraint();


     cout << "[LocalPlanner] finished dynamic objects " << endl;

    isRefUsed = 1;
//    cout<<"Number of Obstacle is "<<obs_q.size()<<endl;
    // LocalPlanner::SetLocalWpts();
     using namespace Eigen;

     //Following codes will be wrapped with another wrapper;
     std::shared_ptr<Problem> prob = std::make_shared<Problem>(box_constraint,obs_q,obs_Q);
     // Do not have to be defined every loop

     bool noConstraint_ = 0;
     prob->set_state_weight(param.state_weight);
     prob->set_final_weight(param.final_weight);
     prob->set_input_weight(param.input_weight);
     prob->setRear_wheel(true);
     prob->set_car_shape(car_shape);
//     prob->setRear_wheel(param.isRearWheeled);
     prob->set_noConstraint(noConstraint_);
     prob->set_refUsed(isRefUsed);
     prob->set_ref(local_wpts);
     prob->set_sfc_idx(sfc_idx);

     static int loop_num = 0;
     std::array<Matrix<double,Nu,1>,N> u0;
     for(auto &s :u0)
     {
     	s=(Matrix<double,Nu,1>()<< 0,0.0).finished();
     }
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

             x0_new = (Matrix<double,Nx,1>()<<p_base->getCarState().x, p_base->getCarState().y,p_base->getCarState().v,
                     0.0, 0.0, p_base->getCarState().theta).finished();
             //cout<<"current x: " <<p_base->getCarState().x<<endl;
             //cout<<"current y: " <<p_base->getCarState().y<<endl;
             //cout<<"current v: " <<p_base->getCarState().v<<endl;
             //cout<<"current theta: " <<p_base->getCarState().theta*180/3.1415926535<<"[deg]"<<endl;
             uN_new = u0;
             iLQR<Nx,Nu,N> ilqr_init(*prob, x0_new,uN_new,param.tStep,ilqr_param);
             ilqr_init.solve();

             //uN_NextInit = ilqr_init.uN_;
             uN_new = ilqr_init.uN_;
             xN_new = ilqr_init.xN_;
             for(int j = 0; j<N;j++)
             {
                 if(xN_new[j][3]>param.maxAccel)
                     xN_new[j][3]=param.maxAccel;
                 if(xN_new[j][3]<param.minAccel)
                     xN_new[j][3]=param.minAccel;
                 if(xN_new[j][4]>param.maxSteer)
                     xN_new[j][4]=param.maxSteer;
                 if(xN_new[j][4]<-param.maxSteer)
                     xN_new[j][4]=-param.maxSteer;

             }
             next_state = xN_new[1];
             
             for(int j = 0;j<N-1;j++)
             {
                 uN_NextInit[j] = uN_new[j+1];
             }
             uN_NextInit[N-1]= uN_new[N-1];

             Matrix<double,N,1> ts_temp = VectorXd::LinSpaced(N,0.0,4.9);
             ts_temp = ts_temp.array()+ t;
             CarState carState_temp;
             CarInput carInput_temp;

             curPlanning.ts.clear();
             curPlanning.xs.clear();
             curPlanning.us.clear();
             for(int i = 0 ;i<N; i++) {
                 carState_temp.x = xN_new[i].coeffRef(0, 0);
                 carState_temp.y = xN_new[i].coeffRef(1, 0);
                 carState_temp.v = xN_new[i].coeffRef(2, 0);
                 carState_temp.theta = xN_new[i].coeffRef(5, 0);

                 carInput_temp.alpha = xN_new[i].coeffRef(3, 0);
                 carInput_temp.delta = xN_new[i].coeffRef(4, 0);

                 curPlanning.ts.push_back(ts_temp.coeffRef(i, 0));
                 curPlanning.xs.push_back(carState_temp);
                 curPlanning.us.push_back(carInput_temp);
             }
             loop_num++;
             return true;
         }
         else
         {
             int flag_unstable = 0;
             //  Be executed after initial Loop (loop_num>0)
             x0_new = (Matrix<double,Nx,1>()<<p_base->getCarState().x, p_base->getCarState().y,p_base->getCarState().v,
                     next_state[3],next_state[4], p_base->getCarState().theta).finished();
             iLQR<Nx,Nu,N> ilqr(*prob, x0_new,uN_NextInit,dt,ilqr_param);
             ilqr.solve();
             uN_new = ilqr.uN_;
             xN_new = ilqr.xN_;

             for(int jj = 0; jj<50;jj++)
             {
                 //cout<<"Future "<<jj<<"th Accel input is"<<xN_new[jj][3]<<endl;
                 if (xN_new[jj][4]>param.maxSteer+0.5 || xN_new[jj][4]<-param.maxSteer-0.5)
                     flag_unstable =1;

             }



             if (!flag_unstable)
             {

                 for(int j = 0; j<N;j++)
                 {
                     if(xN_new[j][3]>param.maxAccel)
                         xN_new[j][3]=param.maxAccel;
                     if(xN_new[j][3]<param.minAccel)
                         xN_new[j][3]=param.minAccel;
                     if(xN_new[j][4]>param.maxSteer)
                         xN_new[j][4]=param.maxSteer;
                     if(xN_new[j][4]<-param.maxSteer)
                         xN_new[j][4]=-param.maxSteer;

                 }
                 next_state = xN_new[1];

                 for(int j = 0;j<N-1;j++)
                 {
                     uN_NextInit[j] = uN_new[j+1];
                 }
                 uN_NextInit[N-1]= uN_new[N-1];

                 Matrix<double,N,1> ts_temp = VectorXd::LinSpaced(N,0.0,4.9);
                 ts_temp = ts_temp.array()+ t;
                 CarState carState_temp;
                 CarInput carInput_temp;

                 curPlanning.ts.clear();
                 curPlanning.xs.clear();
                 curPlanning.us.clear();
                 for(int i = 0 ;i<N; i++) {
                     carState_temp.x = xN_new[i].coeffRef(0, 0);
                     carState_temp.y = xN_new[i].coeffRef(1, 0);
                     carState_temp.v = xN_new[i].coeffRef(2, 0);
                     carState_temp.theta = xN_new[i].coeffRef(5, 0);

                     carInput_temp.alpha = xN_new[i].coeffRef(3, 0);
                     carInput_temp.delta = xN_new[i].coeffRef(4, 0);

                     curPlanning.ts.push_back(ts_temp.coeffRef(i, 0));
                     curPlanning.xs.push_back(carState_temp);
                     curPlanning.us.push_back(carInput_temp);
                 }
                return true;
             }
             else
             {
                uN_NextInit =u0;

                ROS_INFO(" Initial guess =  initialized to zero");
                 return false;
             }
         }

     }



         // Update CarState and CarInput into the form designed in PlannerCore header file
     //loop_num++;
    //TODO: print out the outcome of the planning
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








