#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>
#include <optimization_module/dimension.h>
#include <optimization_module/parameters/dyn_parameter.h>
#include <atypical_planner/PlannerCore.h>
#include <chrono>
#include <iostream>


const double t_stop = 5.0;

namespace Planner{

    class Stopping : public AbstractPlanner{
        private:
            Eigen::Vector4d state0; 
            Eigen::Vector3d goal;
            double thres_;
            std::vector<Eigen::Matrix<double, 2, 1>> u_result;
            const int step_size = 25;
            double pi = 3.141592;

        protected:
            StoppingResult stop_result; 

        public:
            Stopping(shared_ptr<PlannerBase> p_base_): AbstractPlanner(p_base_)
            {
                Matrix<double, 4, 1> start_t = (Matrix<double,4,1>()<<p_base_->getCarState().x, p_base_->getCarState().y,
                    p_base_->getCarState().v, p_base_->getCarState().theta).finished();
                
                Matrix<double, 3, 1> goal_t = (Matrix<double,3,1>()<<p_base_->goal_x, p_base_->goal_y, 0.0).finished();

                state0 = start_t;
                goal = goal_t;

                // bool is_solved = plan(t, state_, goal_);
                // p_base_->isStoppingSolved = is_solved;

            }
            // void set_result(double t);

            bool plan(double time)
            {
                // double x0, y0, v0, a0, del0, th0, j0, sdot0, xf, yf, vf, af, thf;
                // x0 = start_(0,0); y0 = start_(1,0); v0 = start_(2,0); th0 = start_(3,0); 
                // xf = goal_(0,0); yf = goal_(1,0); vf = 0.0; thf = goal_(2,0);

                USING_NAMESPACE_ACADO

                DifferentialState x, y, v, th;
                Control u("", 2,1);

                DifferentialEquation f;

                f << dot(x) == v*cos(th);
                f << dot(y) == v*sin(th);
                f << dot(v) == u(0);
                f << dot(th) == v*tan(u(1))*invL;

                const double t_start = time; const double t_end = time + t_stop; 
                // const double time_time = time;
                OCP ocp(t_start, t_end, step_size);

                ocp.minimizeMayerTerm( u.transpose()*u );
                ocp.subjectTo(f);
                ocp.subjectTo( AT_START, x == state0(0,0));
                ocp.subjectTo( AT_START, y == state0(1,0));
                ocp.subjectTo( AT_START, v == state0(2,0));
                // ocp.subjectTo( AT_START, a == a0);
                // ocp.subjectTo( AT_START, del == del0);
                ocp.subjectTo( AT_START, th == state0(3,0));

                ocp.subjectTo( AT_END, goal(0,0)-thres_ <= x <= goal(0,0)+thres_);
                ocp.subjectTo( AT_END, goal(1,0)-thres_ <= y <= goal(1,0)+thres_);
                ocp.subjectTo( AT_END, v == 0.0);
                ocp.subjectTo( AT_END, u(0) == 0.0);
                ocp.subjectTo(AT_END, th == goal(2,0));

                ocp.subjectTo( steer_min     <= u(1) <= steer_max );
                ocp.subjectTo( acc_min <= u(0) <= acc_max );

                OptimizationAlgorithm alg(ocp);
                alg.set( MAX_NUM_ITERATIONS, 1000 );
                alg.set( PLOT_RESOLUTION, MEDIUM );

                VariablesGrid uStart( 2, t_start, t_end, 2 );
                // uStart( 0,0 ) = j0;
                // uStart( 1,0 ) = sdot0;

                alg.initializeControls(uStart);

                // GnuplotWindow window;
                // window.addSubplot( x, "X [m]" );
                // window.addSubplot( y, "Y [m]" );
                // window.addSubplot( v, "Velocity [m/s]" );
                // window.addSubplot( th,  "Theta [rad]" );
                // window.addSubplot( u(0),  "Accel [m/s^2]" );
                // window.addSubplot( u(1),  "Steer [rad]" );
                            
                // alg << window;
                alg.solve();
                alg.getControls(uStart);

                VariablesGrid controls;
                alg.getControls          (controls  );

                std::vector<DMatrix> control_vec; 
                DMatrix control_mat;
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

                    carInput_temp.alpha = u_result[i].coeffRef(0,0);
                    carInput_temp.delta = u_result[i].coeffRef(1,0);

                    state_temp = mini_dyn(state_temp, u_result[i], dt);

                    stop_result.ts.push_back(ts_temp.coeffRef(i,0));
                    stop_result.xs.push_back(carState_temp);
                    stop_result.us.push_back(carInput_temp);
                }
            }

            bool isCurTrajFeasible() {
                return true;
            }

            void updateStoppingToBase(){
                p_base->setStoppingTraj(stop_result);
            }
    };

}