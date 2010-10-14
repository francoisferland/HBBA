#include "script_engine/publisher_base.hpp"
#include "script_engine/EvalScript.h"
#include <std_msgs/String.h>
#include <pluginlib/class_list_macros.h>

namespace pub_test {

	extern void afun(const v8::Arguments& args, 
		std_msgs::String& msg)
	{
		v8::String::Utf8Value v(args[0]);
		const char* val = *v;
		msg.data = val;
	}

	const extern char test_topic_str[] = "test_topic";
	const extern char pub_test_str[] = "pub_test";
	typedef script_engine::publisher_base<
		std_msgs::String,
		test_topic_str,	
		pub_test_str,	
		&pub_test::afun> pub_test_plugin;

}

PLUGINLIB_DECLARE_CLASS(script_engine, PubPlugin, 
	pub_test::pub_test_plugin,
	script_engine::engine_module);

