#ifndef STOPPING_H
#define STOPPING_H
#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>
#include <optimization_module/dimension.h>
#include <optimization_module/parameters/dyn_parameter.h>
#include <atypical_planner/PlannerCore.h>
#include <chrono>
#include <ros/ros.h>
#include <iostream>


const double t_stop = 5.0;

namespace Planner{

    class Stopping : public AbstractPlanner{
        private:
            Eigen::Vector4d state0; 
            Eigen::Vector3d goal;
            double thres_ = 0.5;
            std::vector<Eigen::Matrix<double, 2, 1>> u_result;
            const int step_size = 10;
            double pi = 3.141592;

        protected:
            StoppingResult stop_result; 

        public:
            Stopping(shared_ptr<PlannerBase> p_base_): AbstractPlanner(p_base_)
            {                
                
                // bool is_solved = plan(t, state_, goal_);
                // p_base_->isStoppingSolved = is_solved;

            }
            // void set_result(double t);

            bool plan(double time)
            {   
                state0 = (Matrix<double,4,1>()<<p_base->getCarState().x, p_base->getCarState().y,
                    p_base->getCarState().v, p_base->getCarState().theta).finished();
                Vector3d goalXYSNU(p_base->goal_x, p_base->goal_y, 0); // goal w.r.t SNu frame
                goalXYSNU = p_base->Tws.inverse()*goalXYSNU;
                goal = goalXYSNU;

                double t_start = time; double t_end = time + 5.0; 

                ROS_INFO("Start Point x: %f, y: %f, v: %f, th: %f", state0(0,0), state0(1,0), state0(2,0), state0(3,0));
                ROS_INFO("Goal Point x: %f, y: %f", goal(0,0), goal(1,0));
                
                 
                double x0, y0, v0, th0, xf, yf;
                x0 = state0(0,0); y0 = state0(1,0); v0 = state0(2,0); th0 = state0(3,0);
                xf = goal(0,0); yf = goal(1,0);
                // x0 = start_(0,0); y0 = start_(1,0); v0 = start_(2,0); th0 = start_(3,0); 
                // xf = goal_(0,0); yf = goal_(1,0); vf = 0.0; thf = goal_(2,0);

                
                ACADO::DifferentialState x, y, v, th;
                ACADO::Control u("", 2,1);

                ACADO::DifferentialEquation f;

                f << dot(x) == v*cos(th);
                f << dot(y) == v*sin(th);
                f << dot(v) == u(0);
                f << dot(th) == v*tan(u(1))*invL;

                // const double time_time = time;
                ACADO::OCP ocp(t_start, t_end, 10);

                ocp.minimizeLagrangeTerm( u.transpose()*u );
                ocp.subjectTo(f);
                ocp.subjectTo( ACADO::AT_START, x == x0);
                ocp.subjectTo( ACADO::AT_START, y == y0);
                ocp.subjectTo( ACADO::AT_START, v == v0);
                ocp.subjectTo( ACADO::AT_START, th == th0);

                ocp.subjectTo( ACADO::AT_END, x == xf);
                ocp.subjectTo( ACADO::AT_END, y == yf);
                // ocp.subjectTo( AT_END, v == 0.0);
                // ocp.subjectTo( AT_END, u(0) == 0.0);

                ocp.subjectTo( steer_min     <= u(1) <= steer_max );
                ocp.subjectTo( acc_min <= u(0) <= acc_max );
                

                ACADO::OptimizationAlgorithm alg(ocp);
                alg.set( ACADO::MAX_NUM_ITERATIONS, 20 );
                alg.set( ACADO::PRINTLEVEL, ACADO::DEBUG );
                alg.set( ACADO::INTEGRATOR_TYPE, ACADO::INT_RK78);
                alg.set( ACADO::INTEGRATOR_TOLERANCE, 1e-8);
                alg.set( ACADO::DISCRETIZATION_TYPE, ACADO::MULTIPLE_SHOOTING);
                alg.set( ACADO::KKT_TOLERANCE, 1e-4);
                // alg.set( QP_SOLVER, QP_FORCES);
                alg.set( ACADO::SPARSE_QP_SOLUTION, ACADO::CONDENSING);

                ACADO::VariablesGrid uStart( 2, t_start, t_end, 2 );
                uStart( 0,0 ) = 0.0;
                uStart( 1,0 ) = 0.0;
                alg.initializeControls(uStart);

                ROS_INFO("[OPT] Start Point x: %f, y: %f, v: %f, th: %f", x0, y0, v0, th0);
                ROS_INFO("[OPT] Goal Point x: %f, y: %f", xf, yf);
                ROS_INFO("[OPT] LIMIT steer_min: %f, steer_max: %f, acc_min: %f, acc_max: %f", steer_min, steer_max, acc_min, acc_max);
                ROS_INFO("[OPT] inv_L: %f", invL);
                ROS_INFO("Time Span start: %f, end: %f", t_start, t_end);
                ROS_INFO("Step size: %d", step_size);

                
                alg.solve();
                ROS_INFO("OPT SUCCESS");
                // GnuplotWindow window;
                // window.addSubplot( x, "X [m]" );
                // window.addSubplot( y, "Y [m]" );
                // window.addSubplot( v, "Velocity [m/s]" );
                // window.addSubplot( th,  "Theta [rad]" );
                // window.addSubplot( u(0),  "Accel [m/s^2]" );
                // window.addSubplot( u(1),  "Steer [rad]" );
                            
                // alg << window;
                alg.getControls(uStart);
                uStart.print();
                

                ACADO::VariablesGrid controls;
                alg.getControls          (controls  );


                std::vector<ACADO::DMatrix> control_vec; 
                ACADO::DMatrix control_mat;
                for(int i=0; i< controls.getNumPoints(); i++){
                    control_mat = controls.getMatrix(i);
                    control_vec.push_back(control_mat);
                }
                // std::cout << typeid(x).name() << std::endl;
                for(int i=0; i<control_vec.size(); i++)
                {
                    std::cout << control_vec.at(i) << std::endl;        
                }

                // std::vector<Eigen::Matrix<double, 2, 1>> return_vec;
                Eigen::Matrix<double, 2, 1> vec;

                for(int i=0; i<control_vec.size(); i++)
                {
                    vec(0,0) = (control_vec.at(i))(0,0);
                    vec(1,0) = (control_vec.at(i))(1,0);
                    u_result.push_back(vec);        
                }
                

                set_result(time);

                if(!u_result.empty())
                    return true;
                return false;
            }
            
            Matrix<double, 4, 1> mini_dyn(Matrix<double, 4, 1>x_, Matrix<double, 2, 1>u_, double dt)
            {   
                double x = x_(0,0); double y = x_(1,0);
                double v = x_(2,0); double th = x_(3,0);
                double acc = u_(0,0); double del = u_(1,0);

                Matrix<double,4,1> dyn;
                dyn.setZero();
                dyn(0,0) = v*std::cos(th);
                dyn(1,0) = v*std::sin(th);
                dyn(2,0) = acc;
                dyn(3,0) = v*std::tan(del)*invL;

                Matrix<double,4,1> next_x;
                next_x = x_ + dyn*dt;
                return next_x;
            }

            void set_result(double t){
                double t_step = dt * N/step_size; 
                stop_result.ts.clear();
                stop_result.us.clear();
                stop_result.xs.clear();
                Matrix<double, 25, 1> ts_temp = VectorXd::LinSpaced(step_size, 0.0, t_stop - t_step);
                ts_temp = ts_temp.array() + t;
                CarState carState_temp;
                CarInput carInput_temp;
                Matrix<double,4,1> state_temp = state0;
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