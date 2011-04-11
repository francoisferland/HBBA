#include "script_engine/engine_module.hpp"
#include <pluginlib/class_list_macros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>

namespace nav_script_engine_plugins {

	class navigation_plugins: public script_engine::engine_module
	{
	public:
		virtual void init(v8::Handle<v8::ObjectTemplate>& global)
		{		
			using namespace v8;
			global->Set(v8::String::New("nav_goto"), 
				v8::FunctionTemplate::New(&navigation_plugins::js_goto));
			global->Set(v8::String::New("nav_goto_state"), 
				v8::FunctionTemplate::New(&navigation_plugins::js_state));

			ac_ = new MoveBaseClient("move_base", true);
		}

		virtual ~navigation_plugins()
		{
			delete ac_;
		}

	private:
		static v8::Handle<v8::Value> js_goto(const v8::Arguments& args)
		{	
			v8::String::Utf8Value frame(args[0]);
			v8::Number* x = v8::Number::Cast(*args[1]);
			v8::Number* y = v8::Number::Cast(*args[2]);
			v8::Number* t = v8::Number::Cast(*args[3]);

			move_base_msgs::MoveBaseGoal g;
			g.target_pose.header.frame_id = *frame;
			g.target_pose.header.stamp = ros::Time::now();
			g.target_pose.pose.position.x = x->Value();
			g.target_pose.pose.position.y = y->Value();
			g.target_pose.pose.orientation = 
				tf::createQuaternionMsgFromYaw(t->Value());

			ac_->sendGoal(g);
			//ac_->waitForResult();

			return v8::Undefined();
		}

		static v8::Handle<v8::Value> js_state(const v8::Arguments& args)
		{	
			//v8::String::Utf8Value nv(args[0]);
			//const char* name = *nv;
			v8::Handle<v8::Boolean> r = v8::Boolean::New(
				ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
			return r;
		}

		typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
			MoveBaseClient;
		static MoveBaseClient* ac_; 

	};

	navigation_plugins::MoveBaseClient* navigation_plugins::ac_;

}

PLUGINLIB_DECLARE_CLASS(nav_script_engine_plugins, Navigation, 
	nav_script_engine_plugins::navigation_plugins,
	script_engine::engine_module);

