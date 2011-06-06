#include "script_engine/engine_module.hpp"
#include "topic_filters/SetState.h"
#include "topic_filters/SetDivisorRate.h"
#include <pluginlib/class_list_macros.h>
#include <topic_filters_manager/manager.hpp>

namespace topic_filters_manager
{
	class ScriptEnginePlugins: public script_engine::engine_module
	{
	public:
		virtual void init(v8::Handle<v8::ObjectTemplate>& global)
		{		
			using namespace v8;
			global->Set(v8::String::New("activateFilter"), 
				v8::FunctionTemplate::New(
                    &ScriptEnginePlugins::js_tfmcall<true>));
			global->Set(v8::String::New("deactivateFilter"), 
				v8::FunctionTemplate::New(
                    &ScriptEnginePlugins::js_tfmcall<false>));
            global->Set(v8::String::New("setDivisorRate"),
                v8::FunctionTemplate::New(
                    &ScriptEnginePlugins::js_ratecall));
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

        static v8::Handle<v8::Value> js_ratecall(const v8::Arguments& args)
        {
            v8::String::Utf8Value nv(args[0]);
            const char* name = *nv;
            v8::Local<v8::Value> rv = args[1];
            v8::Integer* v = v8::Integer::Cast(*rv);
            int rate = v->Value();

            topic_filters::SetDivisorRate req;
            req.request.divisor = rate;
            tfm_.call_filter(name, req);

            return v8::Undefined();
        }

		typedef topic_filters_manager::manager manager_t;
		static manager_t tfm_;

	};

	ScriptEnginePlugins::manager_t ScriptEnginePlugins::tfm_; 

}

PLUGINLIB_DECLARE_CLASS(topic_filters_manager, ScriptEnginePlugins, 
	topic_filters_manager::ScriptEnginePlugins,
	script_engine::engine_module);

