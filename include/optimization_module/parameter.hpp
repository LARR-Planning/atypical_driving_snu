#ifndef PARAMETER_HPP
#define PARAMETER_HPP

#include <optimization_module/utils/parameter.hpp>

class Parameter
{
	private:
		shared_ptr<ParameterHandler> handle;

		struct Setting {
			int verbosity;
			VectorXd state_weight;
			VectorXd input_weight;
			VectorXd final_weight;
			double phi;
			double dphi;
			double dmu;
		};

	public:
		Parameter()
		{ 
			handle = make_shared<ParameterHandler>( "atypical_planning_test" );
			handle->get("setting/verbosity",			setting.verbosity);
			handle->get("setting/weight/state",			setting.state_weight);
			handle->get("setting/weight/input",			setting.input_weight);
			handle->get("setting/weight/final",		setting.final_weight);
			handle->get("setting/phi", 					setting.phi);
			handle->get("setting/dphi",					setting.dphi);
			handle->get("setting/dmu", 					setting.dmu);
		}
		Setting 	setting;
};

#endif
