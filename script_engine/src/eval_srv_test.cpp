#include "script_engine/service_caller_base.hpp"
#include "script_engine/EvalScript.h"
#include <pluginlib/class_list_macros.h>

namespace eval_script_test {

	void afun(const v8::Arguments& args, 
		script_engine::EvalScript::Request& req)
	{
	}

	v8::Handle<v8::Value> rfun(const script_engine::EvalScript::Response& res)
	{
		return v8::True();
	}

	const extern char eval_script_str[] = "eval_script";
	const extern char call_eval_str[] = "call_eval";
	typedef script_engine::service_caller_base<
		script_engine::EvalScript,
		eval_script_str,	
		call_eval_str,	
		afun,
		rfun> call_eval_plugin;

}

PLUGINLIB_DECLARE_CLASS(script_engine, EvalPlugin, 
	eval_script_test::call_eval_plugin,
	script_engine::engine_module);

