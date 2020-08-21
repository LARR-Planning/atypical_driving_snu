// #include <third_party/stopping.h>


// std::vector<Eigen::Matrix<double, 2, 1>> Planner::Stopping::solve_optimal_control()
// {   
//     double x0, y0, v0, a0, del0, th0, j0, sdot0, xf, yf, vf, af, thf;
//     x0 = start_(0,0); y0 = start_(1,0); v0 = start_(2,0); a0 = start_(3,0); del0 = start_(4,0); th0 = start_(5,0); 
//     j0 = input_(0,0); sdot0 = input_(1,0);
//     xf = goal_(0,0); yf = goal_(1,0); vf = 0.0; af = 0.0; thf = goal_(2,0);

//     USING_NAMESPACE_ACADO

//     DifferentialState x, y, v, a, del, th;
//     Control u("", 2,1);

//     DifferentialEquation f;

//     f << dot(x) == v*cos(th);
//     f << dot(y) == v*sin(th);
//     f << dot(v) == a;
//     f << dot(a) == u(0);
//     f << dot(del) == u(1);
//     f << dot(th) == v*tan(del)*invL;


//     OCP ocp(t_start, t_end, 25);

//     ocp.minimizeMayerTerm( u.transpose()*u );
//     ocp.subjectTo(f);
//     ocp.subjectTo( AT_START, x == x0);
//     ocp.subjectTo( AT_START, y == y0);
//     ocp.subjectTo( AT_START, v == v0);
//     ocp.subjectTo( AT_START, a == a0);
//     ocp.subjectTo( AT_START, del == del0);
//     ocp.subjectTo( AT_START, th == th0);

//     ocp.subjectTo( AT_END, x == xf);
//     ocp.subjectTo( AT_END, y == yf);
//     ocp.subjectTo( AT_END, v == vf);
//     ocp.subjectTo( AT_END, a == af);
//     ocp.subjectTo( AT_END, th == thf);

//     ocp.subjectTo( steer_min     <= del <= steer_max );
// 	ocp.subjectTo( acc_min <= a <= acc_max );

//     ocp.subjectTo( jerk_min     <= u(0) <= jerk_max );
// 	ocp.subjectTo( steer_dot_min <= u(1) <= steer_dot_max );

//     OptimizationAlgorithm alg(ocp);
// 	alg.set( MAX_NUM_ITERATIONS, 1000 );
// 	alg.set( PLOT_RESOLUTION, MEDIUM );

//     VariablesGrid uStart( 2,0.0, 10.0,2 );
// 	uStart( 0,0 ) = j0;
// 	uStart( 1,0 ) = sdot0;

//     alg.initializeControls(uStart);

//     GnuplotWindow window;
// 	  window.addSubplot( x, "X [m]" );
// 	  window.addSubplot( y, "Y [m]" );
// 	  window.addSubplot( v, "Velocity [m/s]" );
// 	  window.addSubplot( a, "Acceleration [m/s^2]" );
// 	  window.addSubplot( del,  "Delta [rad]" );
// 	  window.addSubplot( th,  "Theta [rad]" );
//       window.addSubplot( u(0),  "Jerk [m/s^3]" );
//       window.addSubplot( u(1),  "Delta Dot [rad/s]" );
	
//     alg << window;
//     alg.solve();
//     alg.getControls(uStart);

//     VariablesGrid controls;
//     // alg.getDifferentialStates(states    );
//     // alg.getParameters        (parameters);
//     alg.getControls          (controls  );
//     // uStart.getValuesSubGrid(0, 30)

//     std::vector<DMatrix> control_vec; 
//     DMatrix control_mat;
//     for(int i=0; i< controls.getNumPoints(); i++){
//         control_mat = controls.getMatrix(i);
//         control_vec.push_back(control_mat);
//     }
//     // std::cout << typeid(x).name() << std::endl;
//     for(int i=0; i<control_vec.size(); i++)
//     {
//         std::cout << control_vec.at(i) << std::endl;        
//     }

//     std::vector<Eigen::Matrix<double, 2, 1>> return_vec;
//     Eigen::Matrix<double, 2, 1> vec;

//     for(int i=0; i<control_vec.size(); i++)
//     {
//         vec(0,0) = (control_vec.at(i))(0,0);
//         vec(1,0) = (control_vec.at(i))(1,0);
//         return_vec.push_back(vec);        
//     }

//     return return_vec; 
// }
