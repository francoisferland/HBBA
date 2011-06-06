#include "script_engine/service_caller_base.hpp"
#include "script_engine/publisher_topic_arg_base.hpp"
#include "script_engine/EvalScript.h"
#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>
#include <stdlib.h>
#include <string>

/// \brief Common functions plugin for the script engine.
namespace common_plugins 
{
    /// \brief Eval function to interpret JS code.
    ///
    /// Note: This actually calls the eval_script service, mostly meant as a 
    /// test of the service caller template.
    namespace eval
    {

        void afun(const v8::Arguments& args, 
            script_engine::EvalScript::Request& req)
        {
        }

        v8::Handle<v8::Value> rfun(
            const script_engine::EvalScript::Response& res)
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

    /// \brief Run commands on the shell.
    class SysCall: public script_engine::engine_module
    {
    public:
        void init(v8::Handle<v8::ObjectTemplate>& global)
        {
            using namespace v8;
            global->Set(String::New("sys"),
                FunctionTemplate::New(SysCall::call));
        }

    private:
        static v8::Handle<v8::Value> call(const v8::Arguments& args)
        {
            using namespace v8;

			String::Utf8Value cmd_v(args[0]);
			const char* cmd = *cmd_v;

            int r = system(cmd);
            return Integer::New(r);

        }

    };

    /// \brief A basic function to publish strings on topics.
    namespace pub_string
    {
        extern void afun(const v8::Arguments& args, 
            std_msgs::String& msg)
        {
            v8::String::Utf8Value v8_text(args[1]);
            msg.data = std::string(*v8_text);
        }

        const extern char pub_name[] = "pubString";

        typedef script_engine::publisher_topic_arg_base<
            std_msgs::String,
            pub_name,	
            afun> PubStringPlugin;
        }
}

PLUGINLIB_DECLARE_CLASS(script_engine, EvalCall, 
	common_plugins::eval::call_eval_plugin,
	script_engine::engine_module);
PLUGINLIB_DECLARE_CLASS(script_engine, SysCall, 
	common_plugins::SysCall,
	script_engine::engine_module);
PLUGINLIB_DECLARE_CLASS(script_engine, PubString, 
	common_plugins::pub_string::PubStringPlugin,
	script_engine::engine_module);

