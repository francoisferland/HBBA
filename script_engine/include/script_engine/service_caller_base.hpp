#ifndef SERVICE_CALLER_BASE_HPP
#define SERVICE_CALLER_BASE_HPP

#include "engine_module.hpp"
#include <boost/function.hpp>
#include <string>
#include <ros/ros.h>

namespace script_engine
{
	/// \brief A template class for service calling engine module.
	///
	/// For a usage sample, see eval_srv_test.cpp.
	template<
		class T, 		// Service type
		const char SName[],	// Service name
		const char FName[],	// Script function name
		// Argument conversion function
		void AFun(const v8::Arguments&, typename T::Request&),
		// Result conversion function
		v8::Handle<v8::Value> RFun(const typename T::Response&)>
	class service_caller_base: public engine_module
	{
	public:
		/// \brief Constructor.
		service_caller_base() 
		{
			ros::NodeHandle n;
			srv_client_ = n.serviceClient<T>(SName);
		}

		virtual void init(v8::Handle<v8::ObjectTemplate>& global)
		{
			using namespace v8;
			global->Set(String::New(FName),
				FunctionTemplate::New(&this_t::call));
		}

	private:
		static v8::Handle<v8::Value> call(const v8::Arguments& args)
		{
			typename T::Request req;
			typename T::Response res;
			AFun(args, req);
			srv_client_.call(req, res);
			return RFun(res);	
		}

		typedef service_caller_base<T, SName, FName, AFun, RFun> this_t;

		// The trick here is that one srv_client_ will be instancied per service
		// type / name pair since it's part of the type.
		static ros::ServiceClient srv_client_;

	};

	template<
		class T,
		const char SName[],
		const char FName[],
		void AFun(const v8::Arguments&, typename T::Request&),
		v8::Handle<v8::Value> RFun(const typename T::Response&)>
	ros::ServiceClient service_caller_base<T, SName, FName, AFun,
		RFun>::srv_client_;
}

#endif

