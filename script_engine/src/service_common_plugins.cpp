#include <script_engine_plugins/service_caller_topic_arg_base.hpp>
#include <hbba_msgs/EvalScript.h>
#include <hbba_msgs/Boolean.h>
#include <hbba_msgs/UpdateRate.h>
#include <pluginlib/class_list_macros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <stdlib.h>
#include <string>

/// \brief Common functions plugin for the script engine.
namespace common_plugins 
{
    /// \brief Eval function to interpret JS code.
    ///
    /// Note: This actually calls the eval_script service, mostly meant as 
    /// test of the service caller template.
    namespace eval
    {

        void afun(const v8::Arguments& args, 
            hbba_msgs::EvalScript::Request& req)
        {
            v8::String::Utf8Value v8_source(args[1]);
            req.source = *v8_source;
        }

        v8::Handle<v8::Value> rfun(
            const hbba_msgs::EvalScript::Response& res)
        {
            return v8::True();
        }

        const extern char call_eval_str[] = "call_eval";
        typedef script_engine_plugins::service_caller_topic_arg_base<
            hbba_msgs::EvalScript,
            call_eval_str,	
            afun,
            rfun> call_eval_plugin;
    }

    /// \brief Call empty service. Call be used for lots of things
    namespace empty
    {

        void afun(const v8::Arguments& args, std_srvs::Empty::Request& req)
        {
        }

        v8::Handle<v8::Value> rfun(const std_srvs::Empty::Response& res)
        {
            return v8::True();
        }

        const extern char call_empty_str[] = "call_empty";
        typedef script_engine_plugins::service_caller_topic_arg_base<
            std_srvs::Empty,
            call_empty_str,	
            afun,
            rfun> call_empty_plugin;
    }

    /// \brief Call boolean service. Call be used for lots of things
    namespace boolean
    {

        void afun(const v8::Arguments& args, 
			hbba_msgs::Boolean::Request& req)
        {
            req.boolean = args[1]->BooleanValue();
        }

        v8::Handle<v8::Value> rfun(
		const hbba_msgs::Boolean::Response& res)
        {
            return v8::True();
        }

        const extern char call_boolean_str[] = "call_boolean";
        typedef script_engine_plugins::service_caller_topic_arg_base<
            hbba_msgs::Boolean,
            call_boolean_str,	
            afun,
            rfun> call_boolean_plugin;
    }


    /// \brief Call service to update a timer rate in a node.
    namespace update_rate
    {

        void afun(const v8::Arguments& args, 
            hbba_msgs::UpdateRate::Request& req)
        {
            v8::Number* v = v8::Number::Cast(*args[1]);
            req.rate = v->Value();
        }

        v8::Handle<v8::Value> rfun(
            const hbba_msgs::UpdateRate::Response& res)
        {
            return v8::True();
        }

        const extern char call_update_rate_str[] = "call_update_rate";
        typedef script_engine_plugins::service_caller_topic_arg_base<
            hbba_msgs::UpdateRate,
            call_update_rate_str,	
            afun,
            rfun> call_update_rate_plugin;
    }

    /// \brief Run commands on the shell.
    class SysCall: public script_engine_plugins::engine_module
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
}

PLUGINLIB_EXPORT_CLASS(
	common_plugins::eval::call_eval_plugin,
	script_engine_plugins::engine_module);
PLUGINLIB_EXPORT_CLASS(
	common_plugins::empty::call_empty_plugin,
	script_engine_plugins::engine_module);
PLUGINLIB_EXPORT_CLASS(
	common_plugins::boolean::call_boolean_plugin,
	script_engine_plugins::engine_module);
PLUGINLIB_EXPORT_CLASS(
	common_plugins::update_rate::call_update_rate_plugin,
	script_engine_plugins::engine_module);
PLUGINLIB_EXPORT_CLASS(
	common_plugins::SysCall,
	script_engine_plugins::engine_module);
