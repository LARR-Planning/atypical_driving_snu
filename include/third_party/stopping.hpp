#ifndef STOPPING_H
#define STOPPING_H
// #include <acado_optimal_control.hpp>
// #include <acado_gnuplot.hpp>
#include <optimization_module/dimension.h>
#include <optimization_module/parameters/dyn_parameter.h>
#include <atypical_planner/PlannerCore.h>
#include <third_party/ct_dynamics.hpp>
#include <ct/optcon/optcon.h>
#include <ct/core/plot/plot.h>
#include <third_party/ct_dynamics.hpp>
#include <chrono>
#include <ros/ros.h>
#include <iostream>


const double t_stop = 5.0;
// using namespace std;
// using namespace ct::core;
// using namespace ct::optcon;

namespace Planner{

    class Stopping : public AbstractPlanner{
        private:
            Eigen::Vector4d state0; 
            Eigen::Vector3d goal;
            double thres_ = 0.5;
            std::vector<Eigen::Matrix<double, 2, 1>> u_result;
            int step_size = 10;
            double time_horizon = 5.0;
            double pi = 3.141592;

            ParamStopping param;
            // iLQRParams ilqr_param; // iLQR parameters
            Eigen::Matrix<double,2,2> car_shape;
            // Collection<Matrix<double,2,1>,N> uN_NextInit;

            // Matrix<double,Nx,1> next_state;
            // Matrix<double,Nx,1> cur_state;
            // bool isRefPlausible; // check whether reference has garbage value

            // bool isAccelSatisfied; // Check MPC results satisfy accel
            // bool isSteerSatisfied; // Check MPC steer satisfy Steer
            // // YW added
            // bool SetLocalWpts(double t); // find closest 50 lane nodes
            // //void QxFromPrediction(Collection<double,N+1> mpcPredictionHeads);        
            // int isRefUsed;
            // Collection<Matrix<double,5,1>,N+1> local_wpts;


            ct::core::Time timeHorizon = 5.0;  // and a final time horizon in [sec]

            Eigen::Matrix<double,Nx,1> state_weight_;
            Eigen::Matrix<double,Nu,1> input_weight_;
            Eigen::Matrix<double,Nx,1> final_weight_;


        protected:
            StoppingResult stop_result; 

        public:
            Stopping(const ParamStopping& s_param, shared_ptr<PlannerBase> p_base_)
            : AbstractPlanner(p_base_), param(s_param)
            {                
                // ilqr_param.rho = 1e-5;
                // ilqr_param.drho = 1.0;
                // ilqr_param.rhoFactor = 1.6;
                // ilqr_param.rhoMax = 1e10;
                // ilqr_param.rhoMin = 1e-6;
                // ilqr_param.tolGrads = power(10.0, VectorXd::LinSpaced(5, -4.0, -6.0));
                // ilqr_param.tolCosts = power(10.0, VectorXd::LinSpaced(5, -2.0, -6.0));
                // ilqr_param.tolConsts = power(10.0, VectorXd::LinSpaced(5, -2.0, -6.0));
                // ilqr_param.alphas = power(10.0, VectorXd::LinSpaced(5, 0.0, -3.0));
                // ilqr_param.maxIter = 1000;
                // ilqr_param.mu = 1.5;
                // ilqr_param.lambda = 0.0;
                // ilqr_param.phi = 0.1;
                // ilqr_param.verbosity = 0;
                // ilqr_param.dmu = 1.2;
                // ilqr_param.dphi = 0.8;
                state0 = (Eigen::Matrix<double,4,1>()<<p_base->getCarState().x, p_base->getCarState().y,
                    p_base->getCarState().v, p_base->getCarState().theta).finished();
                Eigen::Vector3d goalXYSNU(p_base->goal_x, p_base->goal_y, 0); // goal w.r.t SNu frame
                goalXYSNU = p_base->Tws.inverse()*goalXYSNU;
                goal = goalXYSNU;

                car_shape << 2.0, 0.0, 0.0, 2.0;
                state_weight_<< 0.5, 0.5, 0.0, 0.0, 0.0, 0.05;
                final_weight_<< 0.5, 0.5, 0.0, 0.0, 0.0, 1.0;
                input_weight_<< 0.005, 0.5;
                // isRefUsed = 0;
                std::cout << "[Stopping] Init." << std::endl;

                // bool is_solved = plan(t, state_, goal_);
                // p_base_->isStoppingSolved = is_solved;

            }
            // void set_result(double t);
            
            bool plan_ct(double time)
            {   
                const size_t state_dim = 4;
                const size_t control_dim = 2;

                /* STEP 1: set up the Nonlinear Optimal Control Problem
                * First of all, we need to create instances of the system dynamics, the linearized system and the cost function. */

                /* STEP 1-A: create a instance of the oscillator dynamics for the optimal control problem.
                * Please also compare the documentation of SecondOrderSystem.h */
                std::shared_ptr<ct::core::ControlledSystem<state_dim, control_dim>> AtypicalDynamics(
                    new CarDynamics(invL));

                /* STEP 1-B: Although the first order derivatives of the oscillator are easy to derive, let's illustrate the use of the System Linearizer,
                * which performs numerical differentiation by the finite-difference method. The system linearizer simply takes the
                * the system dynamics as argument. Alternatively, you could implement your own first-order derivatives by overloading the class LinearSystem.h */
                std::shared_ptr<ct::core::SystemLinearizer<state_dim, control_dim>> adLinearizer(
                    new ct::core::SystemLinearizer<state_dim, control_dim>(AtypicalDynamics));

                /* STEP 1-F: initialization with initial state and desired time horizon */
                ct::core::StateVector<state_dim> x0;
                x0[0] = state0(0,0);   x0[1] = state0(1,0);  x0[2] = state0(2,0);  x0[3] = state0(3,0);

                /* STEP 1-C: create a cost function. We have pre-specified the cost-function weights for this problem in "nlocCost.info".
                * Here, we show how to create terms for intermediate and final cost and how to automatically load them from the configuration file.
                * The verbose option allows to print information about the loaded terms on the terminal. */
                std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> intermediateCost(
                    new ct::optcon::TermQuadratic<state_dim, control_dim>());
                std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> finalCost(
                    new ct::optcon::TermQuadratic<state_dim, control_dim>());
                bool verbose = true;
                // intermediateCost->setWeights();

                Eigen::MatrixXd Q_inter(4,4); Q_inter = Eigen::Matrix4d::Identity();
                double Q_scale_i = 0.1; double R_scale_i = 0.1;
                Eigen::MatrixXd R_inter(2,2); R_inter = Eigen::Matrix2d::Identity();
                Q_inter = Q_scale_i * Q_inter;
                R_inter = R_scale_i * R_inter;
                intermediateCost->setWeights(Q_inter, R_inter);


                Eigen::MatrixXd Q_final(4,4); Q_final = Eigen::Matrix4d::Identity();
                double Q_scale_f = 1000.0; double R_scale_f = 0.0; 
                Eigen::MatrixXd R_final(2,2); R_final = Eigen::Matrix2d::Identity();
                Q_final = Q_scale_f * Q_final;
                R_final = R_scale_f * R_final;
                finalCost->setWeights(Q_final, R_final);

                Eigen::Vector4d x_f; 
                // x_f[0] = goal(0,0); x_f[1] = goal(1,0); x_f[2] = 0.0; x_f[3] = goal(2,0);
                x_f[0] = x0[0]+ 5.0; x_f[1] = 0.0; x_f[2] = 0.0; x_f[3] = x0[3];

                Eigen::Vector2d u_f; u_f << 0.0, 0.0;
                finalCost->setStateAndControlReference(x_f, u_f);

                std::shared_ptr<ct::optcon::CostFunctionQuadratic<state_dim, control_dim>> costFunction(
                    new ct::optcon::CostFunctionAnalytical<state_dim, control_dim>());
                costFunction->addIntermediateTerm(intermediateCost);
                costFunction->addFinalTerm(finalCost);
                
                /* STEP 1-D: set up the box constraints for the control input*/
                // input box constraint boundaries with sparsities in constraint toolbox format
                // Eigen::VectorXi sp_control(control_dim);
                // sp_control << 1;
                Eigen::VectorXd u_lb(control_dim);
                Eigen::VectorXd u_ub(control_dim);
                u_lb(0) = acc_min; u_lb(1) = steer_min;
                u_ub(0) = acc_max; u_ub(1) = steer_max; 

                // constraint terms
                std::shared_ptr<ct::optcon::ControlInputConstraint<state_dim, control_dim>> controlInputBound(
                    new ct::optcon::ControlInputConstraint<state_dim, control_dim>(u_lb, u_ub));
                controlInputBound->setName("ControlInputBound");

                // input box constraint constraint container
                std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>> inputBoxConstraints(
                    new ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>());

                // add and initialize constraint terms
                inputBoxConstraints->addIntermediateConstraint(controlInputBound, verbose);
                inputBoxConstraints->initialize();
                
                // /* STEP 1-E: set up the box constraints for the states */
                // // state box constraint boundaries with sparsities in constraint toolbox format
                // // we put a box constraint on the velocity, hence the overall constraint dimension is 1.
                // Eigen::VectorXi sp_state(state_dim);
                // sp_state << 0, 1;
                // Eigen::VectorXd x_lb(1);
                // Eigen::VectorXd x_ub(1);
                // x_lb.setConstant(-0.2);
                // x_ub = -x_lb;

                // constraint terms
                // std::shared_ptr<StateConstraint<state_dim, control_dim>> stateBound(
                //     new StateConstraint<state_dim, control_dim>(x_lb, x_ub, sp_state));
                // stateBound->setName("StateBound");

                // input box constraint constraint container
                // std::shared_ptr<ConstraintContainerAnalytical<state_dim, control_dim>> stateBoxConstraints(
                //     new ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>());
                // // add and initialize constraint terms
                // stateBoxConstraints->addIntermediateConstraint(stateBound, verbose);
                // stateBoxConstraints->initialize();


                
                // STEP 1-G: create and initialize an "optimal control problem"
                ct::optcon::ContinuousOptConProblem<state_dim, control_dim> optConProblem(
                    timeHorizon, x0, AtypicalDynamics, costFunction, adLinearizer);

                // add the box constraints to the optimal control problem
                optConProblem.setInputBoxConstraints(inputBoxConstraints);
                // optConProblem.setStateBoxConstraints(stateBoxConstraints);


                std::cout << "STEP1 FINISHED" << std::endl;     

                /* STEP 2: set up a nonlinear optimal control solver. */
                /* STEP 2-A: Create the settings.
                * the type of solver, and most parameters, like number of shooting intervals, etc.,
                * can be chosen using the following settings struct. Let's use, the iterative
                * linear quadratic regulator, iLQR, for this example. In the following, we
                * modify only a few settings, for more detail, check out the NLOptConSettings class. */
                ct::optcon::NLOptConSettings ilqr_settings;
                ilqr_settings.dt = 0.01;  // the control discretization in [sec]
                ilqr_settings.integrator = ct::core::IntegrationType::EULERCT;
                ilqr_settings.discretization = ct::optcon::NLOptConSettings::APPROXIMATION::FORWARD_EULER;
                ilqr_settings.max_iterations = 10;
                ilqr_settings.nThreads = 1;
                ilqr_settings.nlocp_algorithm = ct::optcon::NLOptConSettings::NLOCP_ALGORITHM::GNMS;
                ilqr_settings.lqocp_solver = ct::optcon::NLOptConSettings::LQOCP_SOLVER::HPIPM_SOLVER;  // solve LQ-problems using HPIPM
                ilqr_settings.lqoc_solver_settings.num_lqoc_iterations = 10;                // number of riccati sub-iterations
                ilqr_settings.printSummary = true;

                std::cout << "STEP 2-A Finished" << std::endl; 

                /* STEP 2-B: provide an initial guess */
                // calculate the number of time steps K
                size_t K = ilqr_settings.computeK(timeHorizon);
                /* design trivial initial controller for iLQR. Note that in this simple example,
                * we can simply use zero feedforward with zero feedback gains around the initial position.
                * In more complex examples, a more elaborate initial guess may be required.*/
                ct::core::FeedbackArray<state_dim, control_dim> u0_fb(K, ct::core::FeedbackMatrix<state_dim, control_dim>::Zero());
                ct::core::ControlVectorArray<control_dim> u0_ff(K, ct::core::ControlVector<control_dim>::Zero());
                ct::core::StateVectorArray<state_dim> x_ref_init(K + 1, x0);
                ct::optcon::NLOptConSolver<state_dim, control_dim>::Policy_t initController(x_ref_init, u0_ff, u0_fb, ilqr_settings.dt);

                std::cout << "STEP 2-B Finished" << std::endl; 


                // STEP 2-C: create an NLOptConSolver instance
                ct::optcon::NLOptConSolver<state_dim, control_dim> iLQR(optConProblem, ilqr_settings);
                // set the initial guess
                iLQR.setInitialGuess(initController);

                std::cout << "STEP 2-C Finished" << std::endl; 

                // STEP 3: solve the optimal control problem
                iLQR.solve();

                std::cout << "STEP 3 Finished" << std::endl; 

                // STEP 4: retrieve the solution
                ct::core::StateFeedbackController<state_dim, control_dim> solution = iLQR.getSolution();


            }

//             bool plan(double time)
//             {   
//                 USING_NAMESPACE_ACADO


//                 double t_start = time; double t_end = time + 5.0; 

//                 ROS_INFO("Start Point x: %f, y: %f, v: %f, th: %f", state0(0,0), state0(1,0), state0(2,0), state0(3,0));
//                 ROS_INFO("Goal Point x: %f, y: %f", goal(0,0), goal(1,0));
                
                 
//                 double x0, y0, v0, th0, xf, yf;
//                 x0 = state0(0,0); y0 = state0(1,0); v0 = state0(2,0); th0 = state0(3,0);
// //                xf = goal(0,0); yf = goal(1,0);
//                 xf = x0 + 5.0; yf = y0;
//                 // x0 = start_(0,0); y0 = start_(1,0); v0 = start_(2,0); th0 = start_(3,0);
//                 // xf = goal_(0,0); yf = goal_(1,0); vf = 0.0; thf = goal_(2,0);

                
//                 DifferentialState x, y, v, th;
//                 Control a, del;
//                 // ACADO::IntermediateState i = x+y;

//                 DifferentialEquation f(t_start, t_end);
//                 // std::cout << i<< std::endl;

//                 f << dot(x) == v*cos(th);
//                 f << dot(y) == v*sin(th);
//                 f << dot(v) == a;
//                 f << dot(th) == v*tan(del)*invL;

//                 // const double time_time = time;
//                 u_int mesh = 10;
//                 OCP ocp(t_start, t_end, mesh);

//                 ocp.minimizeMayerTerm( a*a + del*del );
//                 ocp.subjectTo(f);
//                 ocp.subjectTo( AT_START, x == x0);
//                 ocp.subjectTo( AT_START, y == y0);
//                 ocp.subjectTo( AT_START, v == v0);
//                 ocp.subjectTo( AT_START, th == th0);

// //                thres_ = 0.5;
//                 ocp.subjectTo( AT_END, xf-thres_ <= x <= xf+thres_);
//                 ocp.subjectTo( AT_END, yf-thres_ <= y <= yf+thres_);
//                 ocp.subjectTo( AT_END, a == 0.0);
//                 ocp.subjectTo( AT_END, v == 0.0);
//                 ocp.subjectTo( steer_min     <= del <= steer_max );
//                 ocp.subjectTo( acc_min <= a <= acc_max );
                

//                 OptimizationAlgorithm alg(ocp);
//                 alg.init();
//                 // alg.set( ACADO::MAX_NUM_ITERATIONS, 20 );
//                 alg.set( PRINTLEVEL, DEBUG );
//                 // alg.set( ACADO::INTEGRATOR_TYPE, ACADO::INT_RK78);
//                 // alg.set( ACADO::INTEGRATOR_TOLERANCE, 1e-8);
//                 // alg.set( ACADO::DISCRETIZATION_TYPE, ACADO::MULTIPLE_SHOOTING);
//                 // alg.set( ACADO::KKT_TOLERANCE, 1e-4);
//                 // // alg.set( QP_SOLVER, QP_FORCES);
//                 // alg.set( ACADO::SPARSE_QP_SOLUTION, ACADO::CONDENSING);  

//                 VariablesGrid uStart( 2, t_start, t_end, 2 );
//                 // uStart( 0,0 ) = 0.0;
//                 // uStart( 1,0 ) = 0.0;
//                 alg.initializeControls(uStart);

//                 ROS_INFO("[OPT] Start Point x: %f, y: %f, v: %f, th: %f", x0, y0, v0, th0);
//                 ROS_INFO("[OPT] Goal Point x: %f, y: %f", xf, yf);
//                 ROS_INFO("[OPT] LIMIT steer_min: %f, steer_max: %f, acc_min: %f, acc_max: %f", steer_min, steer_max, acc_min, acc_max);
//                 ROS_INFO("[OPT] inv_L: %f", invL);
//                 ROS_INFO("Time Span start: %f, end: %f", t_start, t_end);
//                 ROS_INFO("Step size: %d", step_size);

                
//                 alg.solve();
//                 ROS_INFO("OPT SUCCESS");
//                 // GnuplotWindow window;
//                 // window.addSubplot( x, "X [m]" );
//                 // window.addSubplot( y, "Y [m]" );
//                 // window.addSubplot( v, "Velocity [m/s]" );
//                 // window.addSubplot( th,  "Theta [rad]" );
//                 // window.addSubplot( u(0),  "Accel [m/s^2]" );
//                 // window.addSubplot( u(1),  "Steer [rad]" );
                            
//                 // alg << window;
//                 alg.getControls(uStart);
//                 uStart.print();
                

//                 VariablesGrid controls;
//                 alg.getControls          (controls  );


//                 std::vector<DMatrix> control_vec; 
//                 DMatrix control_mat;
//                 for(int i=0; i< controls.getNumPoints(); i++){
//                     control_mat = controls.getMatrix(i);
//                     control_vec.push_back(control_mat);
//                 }
//                 // std::cout << typeid(x).name() << std::endl;
//                 for(int i=0; i<control_vec.size(); i++)
//                 {
//                     std::cout << control_vec.at(i) << std::endl;        
//                 }

//                 // std::vector<Eigen::Matrix<double, 2, 1>> return_vec;
//                 Eigen::Matrix<double, 2, 1> vec;

//                 for(int i=0; i<control_vec.size(); i++)
//                 {
//                     vec(0,0) = (control_vec.at(i))(0,0);
//                     vec(1,0) = (control_vec.at(i))(1,0);
//                     u_result.push_back(vec);        
//                 }
                

//                 set_result(time);

//                 if(!u_result.empty())
//                     return true;
//                 return false;
//             }

            // bool plan_ilqr(double time)
            // {
            //     cout << "[Stopping] Initialized.. " << endl;
            // //    cout << "[LocalPlanner] Done. " << endl;
            // //    cout<< "I am in the Local Planner plan function"<<endl;
            //     cout<<"---------------------------------"<<endl;
            // //     cout<< "New Local Goal is"<<x_goal_.coeffRef(0,0)<<"and"<<x_goal_.coeffRef(1,0)<<endl;
            //     cout<<"Current car Speed: "<<p_base->getCarState().v<<" [m/s]"<<endl;
            //     cout<<"Current x-position: "<<p_base->getCarState().x<< " [m]"<<endl;
            //     cout<<"Current y-position: "<<p_base->getCarState().y<< " [m]"<<endl;
            //     cout<<"Current heading angle: "<<p_base->getCarState().theta*180/3.1415926535<< " [deg]"<<endl;

            //     isRefPlausible = LocalPlanner::SetLocalWpts(t);
            //     if (!isRefPlausible)
            //         cout << "Reference : abnormal (nan value)"<<endl;

            //     //Following codes will be wrapped with another wrapper;
            //     std::shared_ptr<Problem> prob = std::make_shared<Problem>(box_constraint,obs_q,obs_Q);
            //     // Do not have to be defined every loop

            //     bool noConstraint_ = 0;
            //     prob->set_state_weight(param.state_weight);
            //     prob->set_final_weight(param.final_weight);
            //     prob->set_input_weight(param.input_weight);
            //     prob->set_car_shape(car_shape);
            //     prob->setRear_wheel(param.isRearWheeled);
            //     prob->set_noConstraint(noConstraint_);
            //     prob->set_refUsed(isRefUsed);
            //     prob->set_ref(local_wpts);
            //     prob->set_sfc_idx(sfc_idx);

            //     static int loop_num = 0;
            //     std::array<Matrix<double,Nu,1>,N> u0;
            //     for(int i = 0; i < N; i++)
            //     {
            //         u0[i] = (Matrix<double,Nu,1>()<< 0,0.0).finished();
            //     }
            //     Matrix<double,Nx,1> x0_new;
            //     Collection<Matrix<double,Nu,1>,N> uN_new;
            //     Collection<Matrix<double,Nx,1>,N+1> xN_new;

            //     // if car state contains strange value;
            //     if(isnan(p_base->getCarState().theta) || isnan(p_base->getCarState().theta)|| (p_base->getCarState().v>1000)) {
            //         for (int i = 0; i < N; i++) {
            //             xN_new[i].setZero();
            //             uN_new[i].setZero();
            //         }
            //         xN_new[N].setZero();
            //     }
            //     else{
            //         if(loop_num == 0) //No initial input trajectory. 
            //         {

            //             x0_new = (Matrix<double,Nx,1>()<<p_base->getCarState().x, p_base->getCarState().y,p_base->getCarState().v,
            //                     0.0, 0.0, p_base->getCarState().theta).finished();
            //             //cout<<"current x: " <<p_base->getCarState().x<<endl;
            //             //cout<<"current y: " <<p_base->getCarState().y<<endl;
            //             //cout<<"current v: " <<p_base->getCarState().v<<endl;
            //             //cout<<"current theta: " <<p_base->getCarState().theta*180/3.1415926535<<"[deg]"<<endl;
            //             uN_new = u0;
            //             iLQR<Nx,Nu,N> ilqr_init(*prob, x0_new,uN_new,param.tStep,ilqr_param);
            //             ilqr_init.solve();

            //             //uN_NextInit = ilqr_init.uN_;
            //             uN_new = ilqr_init.uN_;
            //             xN_new = ilqr_init.xN_;
            //             for(int j = 0; j<N;j++)
            //             {
            //                 if(xN_new[j][3]>param.maxAccel)
            //                     xN_new[j][3]=param.maxAccel;
            //                 if(xN_new[j][3]<param.minAccel)
            //                     xN_new[j][3]=param.minAccel;
            //                 if(xN_new[j][4]>param.maxSteer)
            //                     xN_new[j][4]=param.maxSteer;
            //                 if(xN_new[j][4]<-param.maxSteer)
            //                     xN_new[j][4]=-param.maxSteer;

            //             }
            //             next_state = xN_new[1];
                        
            //             for(int j = 0;j<N-1;j++)
            //             {
            //                 uN_NextInit[j] = uN_new[j+1];
            //             }
            //             uN_NextInit[N-1]= uN_new[N-1];

            //             Matrix<double,N,1> ts_temp = VectorXd::LinSpaced(N,0.0,param.horizon-param.tStep);
            //             ts_temp = ts_temp.array()+ t;
            //             CarState carState_temp;
            //             CarInput carInput_temp;

            //             curPlanning.ts.clear();
            //             curPlanning.xs.clear();
            //             curPlanning.us.clear();
            //             for(int i = 0 ;i<N; i++) {
            //                 carState_temp.x = xN_new[i].coeffRef(0, 0);
            //                 carState_temp.y = xN_new[i].coeffRef(1, 0);
            //                 carState_temp.v = xN_new[i].coeffRef(2, 0);
            //                 carState_temp.theta = xN_new[i].coeffRef(5, 0);
            //                 //cout<<"MPC Future Heading Angle"<<i<<"th: "<< carState_temp.theta<<endl;
            //                 carInput_temp.alpha = xN_new[i].coeffRef(3, 0);
            //                 carInput_temp.delta = xN_new[i].coeffRef(4, 0);

            //                 curPlanning.ts.push_back(ts_temp.coeffRef(i, 0));
            //                 curPlanning.xs.push_back(carState_temp);
            //                 curPlanning.us.push_back(carInput_temp);
            //             }
            //             curPlanning.isSuccess = true;
            //             loop_num++;
            //             return true;
            //         }
            // }


            
            Eigen::Matrix<double, 4, 1> mini_dyn(Eigen::Matrix<double, 4, 1>x_, Eigen::Matrix<double, 2, 1>u_, double dt)
            {   
                double x = x_(0,0); double y = x_(1,0);
                double v = x_(2,0); double th = x_(3,0);
                double acc = u_(0,0); double del = u_(1,0);

                Eigen::Matrix<double,4,1> dyn;
                dyn.setZero();
                dyn(0,0) = v*std::cos(th);
                dyn(1,0) = v*std::sin(th);
                dyn(2,0) = acc;
                dyn(3,0) = v*std::tan(del)*invL;

                Eigen::Matrix<double,4,1> next_x;
                next_x = x_ + dyn*dt;
                return next_x;
            }

            void set_result(double t){
                double t_step = dt * N/step_size; 
                stop_result.ts.clear();
                stop_result.us.clear();
                stop_result.xs.clear();
                Eigen::Matrix<double, 10, 1> ts_temp = Eigen::VectorXd::LinSpaced(step_size, 0.0, time_horizon);
                ts_temp = ts_temp.array() + t;
                CarState carState_temp;
                CarInput carInput_temp;
                Eigen::Matrix<double,4,1> state_temp = state0;
                for(int i=0; i< step_size;i++){
                    carState_temp.x = state0.coeffRef(0,0);
                    carState_temp.y = state0.coeffRef(1,0);
                    carState_temp.v = state0.coeffRef(2,0);
                    carState_temp.theta = state0.coeffRef(3,0);

                    carInput_temp.alpha = (u_result.at(i)).coeffRef(0,0);
                    carInput_temp.delta = (u_result.at(i)).coeffRef(1,0);

                    state_temp = mini_dyn(state_temp, (u_result.at(i)), dt);

                    stop_result.ts.push_back(ts_temp.coeffRef(i,0));
                    stop_result.xs.push_back(carState_temp);
                    stop_result.us.push_back(carInput_temp);
                }

                ROS_INFO("SET RESULT END");
            }

            bool isCurTrajFeasible() {
                return true;
            }

            void updateStoppingToBase(){
                p_base->setStoppingTraj(stop_result);
            }
    };

}

#endif