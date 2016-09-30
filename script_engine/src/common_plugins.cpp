#include <script_engine_plugins/service_caller_base.hpp>
#include <script_engine_plugins/publisher_topic_arg_base.hpp>
#include <hbba_msgs/EvalScript.h>
#include <hbba_msgs/Boolean.h>
#include <hbba_msgs/UpdateRate.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
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
        }

        v8::Handle<v8::Value> rfun(
            const hbba_msgs::EvalScript::Response& res)
        {
            return v8::True();
        }

        const extern char eval_script_str[] = "eval_script";
        const extern char call_eval_str[] = "call_eval";
        typedef script_engine::service_caller_base<
            hbba_msgs::EvalScript,
            eval_script_str,	
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

        const extern char empty_script_str[] = "empty_script";
        const extern char call_empty_str[] = "call_empty";
        typedef script_engine::service_caller_base<
            std_srvs::Empty,
            empty_script_str,	
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
            v8::BooleanObject* b = v8::BooleanObject::Cast( *args[1] );
            req.boolean = b->BooleanValue();
        }

        v8::Handle<v8::Value> rfun(
		const hbba_msgs::Boolean::Response& res)
        {
            return v8::True();
        }

        const extern char boolean_script_str[] = "boolean_script";
        const extern char call_boolean_str[] = "call_boolean";
        typedef script_engine::service_caller_base<
            hbba_msgs::Boolean,
            boolean_script_str,	
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

        const extern char update_rate_script_str[] = "update_rate_script";
        const extern char call_update_rate_str[] = "call_update_rate";
        typedef script_engine::service_caller_base<
            hbba_msgs::UpdateRate,
            update_rate_script_str,	
            call_update_rate_str,	
            afun,
            rfun> call_update_rate_plugin;
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

    /// \brief A basic function to publish on std_msgs/Empty topics.
    namespace pub_empty
    {
        extern void afun(const v8::Arguments&, std_msgs::Empty&)
        {
        }

        const extern char pub_name[] = "pubEmpty";

        typedef script_engine::publisher_topic_arg_base<
            std_msgs::Empty,
            pub_name,
            afun> PubEmptyPlugin;
    }
    

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

    /// \brief A geometry_msgs/PoseStamped publication tool takes as input a 
    /// planar (x, y, theta) pose usually meant as a navigation goal.
    ///
    /// Params:
    ///  0: The output topic's name, taken care of by the base class.
    ///  1: The frame id (String)
    ///  2: x position
    ///  3: y position
    ///  4: theta angle, (around Z).
    namespace pub_nav_goal
    {
        extern void afun(const v8::Arguments& args,
            geometry_msgs::PoseStamped& msg)
        {
            if (args.Length() < 5)
            {
                ROS_ERROR("Not enough arguments in pubNavGoal call.");
                return;
            }

            v8::String::Utf8Value frame_id(args[1]);
            v8::Number* vx = v8::Number::Cast(*args[2]);
            v8::Number* vy = v8::Number::Cast(*args[3]);
            v8::Number* vt = v8::Number::Cast(*args[4]);

            ROS_DEBUG("Sending navigation goal: %s, %f, %f, %f.",
                msg.header.frame_id.c_str(),
                vx->Value(),
                vy->Value(),
                vt->Value());

            msg.header.frame_id = *frame_id;
            msg.header.stamp = ros::Time::now();
            msg.pose.position.x = vx->Value();
            msg.pose.position.y = vy->Value();
            msg.pose.position.z = 0.0;
            msg.pose.orientation = tf::createQuaternionMsgFromYaw(vt->Value());

        }

        const extern char pub_name[] = "pubNavGoal";

        typedef script_engine::publisher_topic_arg_base<
            geometry_msgs::PoseStamped,
            pub_name,
            afun> PubNavGoalPlugin;
    }
}

PLUGINLIB_DECLARE_CLASS(script_engine, EvalCall, 
	common_plugins::eval::call_eval_plugin,
	script_engine::engine_module);
PLUGINLIB_DECLARE_CLASS(script_engine, EmptyCall, 
	common_plugins::empty::call_empty_plugin,
	script_engine::engine_module);
PLUGINLIB_DECLARE_CLASS(script_engine, BooleanCall, 
	common_plugins::boolean::call_boolean_plugin,
	script_engine::engine_module);
PLUGINLIB_DECLARE_CLASS(script_engine, UpdateRateCall, 
	common_plugins::update_rate::call_update_rate_plugin,
	script_engine::engine_module);
PLUGINLIB_DECLARE_CLASS(script_engine, SysCall, 
	common_plugins::SysCall,
	script_engine::engine_module);
PLUGINLIB_DECLARE_CLASS(script_engine, PubEmpty,
    common_plugins::pub_empty::PubEmptyPlugin,
    script_engine::engine_module);
PLUGINLIB_DECLARE_CLASS(script_engine, PubString, 
	common_plugins::pub_string::PubStringPlugin,
	script_engine::engine_module);
PLUGINLIB_DECLARE_CLASS(script_engine, PubNavGoal, 
	common_plugins::pub_nav_goal::PubNavGoalPlugin,
	script_engine::engine_module);

