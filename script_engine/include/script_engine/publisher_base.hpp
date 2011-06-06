#ifndef PUBLISHER_BASE_HPP
#define PUBLISHER_BASE_HPP

#include "engine_module.hpp"
#include <boost/function.hpp>
#include <string>
#include <ros/ros.h>

namespace script_engine
{
	/// \brief A template class for publishing to a specific topic.
	///
	/// You can only have one topic type/name/func tuple per node.
	/// For a usage sample, see pub_test.cpp.
	template<
		class T, 		// Topic type
		const char TName[],	// Topic name
		const char FName[],	// Script function name
		// Argument conversion function
		void AFun(const v8::Arguments&, T&)>

	class publisher_base: public engine_module
	{
	public:
		/// \brief Constructor.
		publisher_base() 
		{
			ros::NodeHandle n;
			pub_ = n.advertise<T>(TName, 5);
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
			T msg;
			AFun(args, msg);
			pub_.publish(msg);				
			return v8::True();
		}

		typedef publisher_base<T, TName, FName, AFun> this_t;

		static ros::Publisher pub_;

	};

	template<
		class T,
		const char TName[],
		const char FName[],
		void AFun(const v8::Arguments&, T&)>
	ros::Publisher publisher_base<T, TName, FName, AFun>::pub_; 

}

#endif

