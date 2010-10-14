#include "script_engine/engine_module.hpp"
#include "topic_filters/SetState.h"
#include <pluginlib/class_list_macros.h>
#include <topic_filters_manager/manager.hpp>

namespace iw_translator_v8 {


	class engine_plugins: public script_engine::engine_module
	{
	public:
		virtual void init(v8::Handle<v8::ObjectTemplate>& global)
		{		
			using namespace v8;
			global->Set(v8::String::New("activate"), 
				v8::FunctionTemplate::New(&engine_plugins::js_tfmcall<true>));
			global->Set(v8::String::New("deactivate"), 
				v8::FunctionTemplate::New(&engine_plugins::js_tfmcall<false>));
		}

	private:
		template <const bool State>
		static v8::Handle<v8::Value> js_tfmcall(const v8::Arguments& args)
		{	
			v8::String::Utf8Value nv(args[0]);
			const char* name = *nv;
			topic_filters::SetState req;
			req.request.state = State;
			tfm_.call_filter<topic_filters::SetState>(name, req);
			return v8::Undefined();
		}

		typedef topic_filters_manager::manager manager_t;
		static manager_t tfm_;

	};

	engine_plugins::manager_t engine_plugins::tfm_; 

}

PLUGINLIB_DECLARE_CLASS(iw_translator_v8, TranslatorPlugins, 
	iw_translator_v8::engine_plugins,
	script_engine::engine_module);

