#ifndef PUBLISHER_TOPIC_ARG_BASE_HPP
#define PUBLISHER_TOPIC_ARG_BASE_HPP

#include "engine_module.hpp"
#include <boost/function.hpp>
#include <string>
#include <ros/ros.h>
#include <tr1/unordered_map>

namespace script_engine
{
	typedef std::tr1::unordered_map<std::string,ros::Publisher> pub_map_t;
	/// TODO verify if there is no conflict
	pub_map_t pub_map_;
	/// \brief A template class for publishing to a specific topic.
	///
    /// The first argument of your function is assumed to be the topic name.
	/// You can only have one topic type/name/func tuple per node.
	/// For a usage sample, see pub_test.cpp.
	template<
		class T, 		// Topic type
		const char FName[],	// Script function name
		// Argument conversion function
		void AFun(const v8::Arguments&, T&)>

	class publisher_topic_arg_base: public engine_module
	{
	public:
		/// \brief Constructor.
		publisher_topic_arg_base() 
		{
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
			ros::NodeHandle n;
			v8::String::Utf8Value v8_topic_name(args[0]);
			const char* topic_name = *v8_topic_name;
			ros::Publisher pub;
			pub_map_t::iterator i = pub_map_.find(topic_name);
			if(i == pub_map_.end())
			{
				pub = n.advertise<T>(topic_name, 5);
				pub_map_[std::string(topic_name)] = pub;
				sleep(1);
			}
			else
				pub = i->second;

			T msg;
			AFun(args, msg);

			pub.publish(msg);				
			return v8::True();
		}

		typedef publisher_topic_arg_base<T, FName, AFun> this_t;

	};

}

#endif

