#ifndef UTILS_PARAMETER_HANDLER_HPP
#define UTILS_PARAMETER_HANDLER_HPP

#include <Eigen/Dense>
#include <ros/ros.h>
#include <boost/variant.hpp>

using namespace std;
// using namespace Eigen;

class ParameterHandler
{
	public:
		ParameterHandler( const string node_name )
		{ pnh = make_shared<ros::NodeHandle>( node_name ); }
		void get(const string name, double &dst){ dst = boost::get<double>(get<double>(name)); }
		void get(const string name, int &dst){ dst = boost::get<int>(get<int>(name)); }
		void get(const string name, Eigen::VectorXd &dst){ dst = boost::get<Eigen::VectorXd>(get<double>(name)); }
		void get(const string name, Eigen::VectorXi &dst){ dst = boost::get<Eigen::VectorXi>(get<int>(name)); }
		void get(const string name, string &dst){ dst = get(name); }

	private:
		shared_ptr<ros::NodeHandle> pnh;
	
		template<typename T>
		boost::variant<T,Eigen::Matrix<T,Eigen::Dynamic,1>> get( const string name )
		{
			XmlRpc::XmlRpcValue data;
			if( !pnh->getParam( name, data ) )
			{	// parameter is not found
				print_error( name );
				return -1;
			}
			else
			{	// parameter is found
				if( data.getType() == XmlRpc::XmlRpcValue::Type::TypeArray )
				{	// value is a vector
					Eigen::Matrix<T,Eigen::Dynamic,1> dst = Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic>(data.size(),1);
					for(int i=0; i<data.size(); i++)
						dst(i) = static_cast<T>( data[i] );
					return dst;
				}
				else
				{	// value is a scalar
					T dst = data;
					return dst;
				}
			}
		}
		
		string get( const string name )
		{
			XmlRpc::XmlRpcValue data;
			if( !pnh->getParam( name, data ) )
			{	// parameter is not found
				print_error( name );
				return string("");
			}
			else
			{	// parameter is found
				if( data.getType() == XmlRpc::XmlRpcValue::Type::TypeString )
				{	// value is string
					string dst = data;
					return dst;
				}
				else
				{
					ROS_WARN_STREAM( "Parameter " << name << " is expected as [string]." );
					return string("");
				}
			}

		}

		void print_error( const string &msg )
		{	ROS_WARN_STREAM( "Parameter " << msg << " not found." ); }
};

#endif
