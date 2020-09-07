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
#include <third_party/matplotlibcpp.h>


const double t_stop = 5.0;
// using namespace std;
// using namespace ct::core;
// using namespace ct::optcon;
namespace plt = matplotlibcpp;

namespace Planner{

    class Stopping : public AbstractPlanner{
        private:
            Eigen::Vector4d state0; 
            Eigen::Vector3d goal;
            double thres_ = 0.5;
            std::vector<Eigen::Matrix<double, 2, 1>> u_result;
            int step_size = 50;
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
            
            bool plan(double time)
            {   
                state0 = (Eigen::Matrix<double,4,1>()<<p_base->getCarState().x, p_base->getCarState().y,
                          p_base->getCarState().v, p_base->getCarState().theta).finished();
                Eigen::Vector3d goalXYSNU(p_base->goal_x, p_base->goal_y, 0); // goal w.r.t SNu frame
                goalXYSNU = p_base->Tws.inverse()*goalXYSNU;
                goal = goalXYSNU;

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
                
                Eigen::Vector4d x_f; 
                x_f[0] = goal(0,0); x_f[1] = goal(1,0); x_f[2] = 0.0; x_f[3] = x0[3];

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
                double Q_scale_i = 0.1; double R_scale_i = 100.0;
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

                // x_f[0] = x0[0]+ 5.0; x_f[1] = 0.0; x_f[2] = 0.0; x_f[3] = x0[3];

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

                ROS_INFO("[OPT] Start Point x: %f, y: %f, v: %f, th: %f", x0[0], x0[1], x0[2], x0[3]);
                ROS_INFO("[OPT] Goal Point x: %f, y: %f, th: %f", x_f[0], x_f[1], x_f[3]);


                /* STEP 2: set up a nonlinear optimal control solver. */
                /* STEP 2-A: Create the settings.
                * the type of solver, and most parameters, like number of shooting intervals, etc.,
                * can be chosen using the following settings struct. Let's use, the iterative
                * linear quadratic regulator, iLQR, for this example. In the following, we
                * modify only a few settings, for more detail, check out the NLOptConSettings class. */
                ct::optcon::NLOptConSettings ilqr_settings;
                ilqr_settings.dt = dt;  // the control discretization in [sec]
                ilqr_settings.integrator = ct::core::IntegrationType::EULERCT;
                ilqr_settings.discretization = ct::optcon::NLOptConSettings::APPROXIMATION::FORWARD_EULER;
                ilqr_settings.max_iterations = 200;
                ilqr_settings.nThreads = 1;
                ilqr_settings.nlocp_algorithm = ct::optcon::NLOptConSettings::NLOCP_ALGORITHM::GNMS;
                ilqr_settings.lqocp_solver = ct::optcon::NLOptConSettings::LQOCP_SOLVER::HPIPM_SOLVER;  // solve LQ-problems using HPIPM
                ilqr_settings.lqoc_solver_settings.num_lqoc_iterations = 10;                // number of riccati sub-iterations
                ilqr_settings.printSummary = false;


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



                // STEP 2-C: create an NLOptConSolver instance
                ct::optcon::NLOptConSolver<state_dim, control_dim> iLQR(optConProblem, ilqr_settings);
                // set the initial guess
                iLQR.setInitialGuess(initController);


                // STEP 3: solve the optimal control problem
                iLQR.solve();


                // STEP 4: retrieve the solution
                ct::core::StateFeedbackController<state_dim, control_dim> solution = iLQR.getSolution();

                //Conversion to u_result
                //std::vector<Eigen::Matrix<double, 2, 1>> u_result;
                auto stateArray = solution.x_ref();
                auto controlArray = solution.uff();
                auto timeArray = solution.time();                
                if (timeArray.size() != stateArray.size())
                {
                    std::cout << timeArray.size() << std::endl;
                    std::cout << stateArray.size() << std::endl;
                    std::cout << "Cannot plot data, x and t not equal length" << std::endl;
                    return false; 
                }
                Eigen::Vector2d input; 
                for (size_t j = 0; j < controlArray.size(); j++)
                {
                    input[0] = controlArray[j](0);
                    input[1] = controlArray[j](1);
                    u_result.push_back(input);
                }


                std::cout << "Size of the timeArray: " << timeArray.size() << std::endl; 
                
                
                std::vector<double> x_traj;
                std::vector<double> y_traj;
                std::vector<double> th_traj;
                std::vector<double> v_traj;
                    
                std::vector<double> time_state;
                for (size_t j = 0; j < stateArray.size(); j++)
                {
                    x_traj.push_back(stateArray[j](0));
                    y_traj.push_back(stateArray[j](1));
                    v_traj.push_back(stateArray[j](2));
                    th_traj.push_back(stateArray[j](3));
                    time_state.push_back(timeArray[j]);
                }

                std::vector<double> a_traj;
                std::vector<double> del_traj;
                
                std::vector<double> time_control;
                for (size_t j = 0; j < controlArray.size(); j++)
                {
                    a_traj.push_back(controlArray[j](0));
                    del_traj.push_back(controlArray[j](1));        
                    time_control.push_back(timeArray[j]);
                }

                // plt::figure(1);
                // // plt::figure_size(1500, 1000);

                // std::cout << "Time size: " <<  time_state.size() << std::endl; 
                // std::cout << "X size: " << x_traj.size() << std::endl; 
                // std::cout << "Y size: " << y_traj.size() << std::endl; 
                // std::cout << "V size: " << v_traj.size() << std::endl; 
                // std::cout << "Theta size: " << th_traj.size() << std::endl; 

                // plt::subplot(3, 2, 1);
                // plt::named_plot("x", time_state, x_traj);
                // plt::title("x_traj");

                // plt::subplot(3, 2, 2);
                // plt::named_plot("y", time_state, y_traj);
                // plt::title("y_traj");

                // plt::subplot(3, 2, 3);
                // plt::plot(time_state, v_traj);
                // plt::title("v_traj");

                // plt::subplot(3, 2, 4);
                // plt::plot(time_state, th_traj);
                // plt::title("th_traj");

                // plt::subplot(3, 2, 5);
                // plt::plot(time_control, a_traj);
                // plt::title("a_traj");

                // plt::subplot(3, 2, 6);
                // plt::plot(time_control, del_traj);
                // plt::title("del_traj");


                // plt::legend();
                // plt::show(false);
                // plt::save("/home/dabinnkim/stopping_state.png");


                set_result(time);
                u_result.clear();
                return true; 
            }
            
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
                Eigen::Matrix<double, 50, 1> ts_temp = Eigen::VectorXd::LinSpaced(step_size, 0.0, time_horizon);
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

                    ROS_INFO("Time: %f, Alpha: %f, Delta: %f",ts_temp.coeffRef(i,0), carInput_temp.alpha, carInput_temp.delta);

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