#include <script_engine_plugins/engine_module.hpp>
#include "topic_filters/SetState.h"
#include "topic_filters/SetDividerRate.h"
#include <pluginlib/class_list_macros.h>
#include <topic_filters_manager/manager.hpp>
#include <topic_filters_manager/gd_manager.hpp>

namespace topic_filters_manager
{
	class ScriptEnginePlugins: public script_engine_plugins::engine_module
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
            global->Set(v8::String::New("setDividerRate"),
                v8::FunctionTemplate::New(
                    &ScriptEnginePlugins::js_ratecall));
            global->Set(v8::String::New("setGenericDividerRate"),
                        v8::FunctionTemplate::New(
                            &ScriptEnginePlugins::js_gdratecall));
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

            topic_filters::SetDividerRate req;
            req.request.divider = rate;
            tfm_.call_filter(name, req);

            return v8::Undefined();
        }

        static v8::Handle<v8::Value> js_gdratecall(const v8::Arguments& args)
        {
            v8::String::Utf8Value nv(args[0]);
            const char* name = *nv;
            v8::Local<v8::Value> rv = args[1];
            v8::Integer* v = v8::Integer::Cast(*rv);
            int rate = v->Value();

            gdm_.setRate(name, rate);

            return v8::Undefined();
        }

		typedef topic_filters_manager::manager               manager_t;
		typedef topic_filters_manager::GenericDividerManager gdmanager_t;
		static manager_t   tfm_;
        static gdmanager_t gdm_;

	};

	ScriptEnginePlugins::manager_t   ScriptEnginePlugins::tfm_; 
	ScriptEnginePlugins::gdmanager_t ScriptEnginePlugins::gdm_; 

}

PLUGINLIB_EXPORT_CLASS(topic_filters_manager::ScriptEnginePlugins,
                       script_engine_plugins::engine_module);

