#include <script_engine/engine_v8.hpp>
#include <script_engine_plugins/engine_module.hpp>

using namespace script_engine;

namespace {
	void se_log(const v8::FunctionCallbackInfo<v8::Value>& args)
	{
		v8::Isolate* isolate = args.GetIsolate();
		v8::String::Utf8Value v(args[0]);
		const char* val = *v;
		ROS_INFO("se_log: %s", val);
		args.GetReturnValue().Set(v8::True(isolate));
	}

	void se_error(const v8::FunctionCallbackInfo<v8::Value>& args)
	{
		v8::Isolate* isolate = args.GetIsolate();
		v8::String::Utf8Value v(args[0]);
		const char* val = *v;
		ROS_ERROR("se_error: %s", val);
		args.GetReturnValue().Set(v8::True(isolate));
	}
}

engine_v8::engine_v8():
//	global_(v8::ObjectTemplate::New()),
//	context_(v8::Context::New(NULL, global_)),
//	scope_(v8::Isolate::GetCurrent()),
	isolate(v8::Isolate::New()),
	module_loader_(new module_loader_t("script_engine_plugins",
		"script_engine_plugins::engine_module"))
{
    using namespace hbba_msgs;

	ros::NodeHandle n, np("~");

    np.param("no_eval", no_eval_, false);

	srv_eval_script_ = n.advertiseService("eval_script",
		&engine_v8::eval_srv, this);
	srv_compile_script_ = n.advertiseService("compile_script",
		&engine_v8::compile_srv, this);
	srv_run_script_ = n.advertiseService("run_script",
		&engine_v8::run_srv, this);

        //isolate = v8::Isolate::New();
	v8::Isolate::Scope isolate_scope(isolate);
	v8::HandleScope handle_scope(isolate);
        global_ = v8::ObjectTemplate::New(isolate);

	// Log functions.
	global_->Set(v8::String::NewFromUtf8(isolate,"se_log"),
		v8::FunctionTemplate::New(isolate,se_log));
	global_->Set(v8::String::NewFromUtf8(isolate,"se_error"),
		v8::FunctionTemplate::New(isolate,se_error));

	// Load plugins.
	std::vector<std::string> classes = module_loader_->getDeclaredClasses();
	std::vector<std::string>::const_iterator i;
	for (i = classes.begin(); i != classes.end(); ++i)
	{
		try 
		{
            ROS_INFO("Loading script engine plugin %s...", i->c_str());
			engine_module* m = module_loader_->createUnmanagedInstance(*i);
			m->init(isolate, global_);
			modules_list_.push_back(m);
		}
		catch (pluginlib::LibraryLoadException e)
		{
			ROS_ERROR("Cannot load script_engine plugin, exception: \n %s", 
				e.what());
		}
	}

	context_ = v8::Context::New(isolate,NULL, global_);	
	ROS_WARN("ALLO");
}

engine_v8::~engine_v8()
{
	modules_list_t::iterator i;
	for (i = modules_list_.begin(); i != modules_list_.end(); ++i)
		delete *i;
	v8::V8::Dispose();
}

bool engine_v8::eval(const std::string& src, std::string& result)
{
	ROS_DEBUG("eval(\"%s\") ...", src.c_str());
	using namespace v8;
	Context::Scope context_scope(context_);
	Handle<String> str = String::NewFromUtf8(context_->GetIsolate(),src.c_str());

    Handle<Script> script;
    {
        TryCatch tc;
        script = Script::Compile(str);
        if (tc.HasCaught())
        {
            result = *String::Utf8Value(tc.Message()->Get());
            ROS_ERROR("V8 compile error: %s: %s \n when running: \n\t%s",
                result.c_str(),
                *String::Utf8Value(tc.Message()->GetSourceLine()),
                src.c_str());
            return false;
        }
    }

    if (!script.IsEmpty())
    {
        if (!run_script(script, result))
        {
            ROS_ERROR("Exception when evaluating %s", src.c_str());
            return false;
        }
        return true;
    }
    else
        return false;
}

bool engine_v8::eval_srv(hbba_msgs::EvalScript::Request& req,
                         hbba_msgs::EvalScript::Response& res)
{
    if (no_eval_)
        ROS_INFO("Eval: %s", req.source.c_str());
    else
        eval(req.source, res.result);

	return true;
}

bool engine_v8::compile(const std::string& name, const std::string& src)
{
	using namespace v8;
	Context::Scope context_scope(context_);
	Handle<String> source = String::NewFromUtf8(context_->GetIsolate(),src.c_str());
	scripts_map_[name] = Script::Compile(source);

	return true;
}

bool engine_v8::compile_srv(hbba_msgs::CompileScript::Request& req,
	                        hbba_msgs::CompileScript::Response& res)
{
	return compile(req.name, req.source);
}

bool engine_v8::run(const std::string& name, std::string& result)
{
	v8::Context::Scope context_scope(context_);
	return run_script(scripts_map_[name], result);
	//scripts_map_[name]->Run();

	return true;
}

bool engine_v8::run_srv(hbba_msgs::RunScript::Request& req, 
                        hbba_msgs::RunScript::Response& res)
{
	return run(req.name, res.result);
}

bool engine_v8::run_script(const v8::Handle<v8::Script>& s, std::string& result)
{
	using namespace v8;

	TryCatch tc;
	Handle<Value> v = s->Run();
	if (v.IsEmpty())
	{
		Handle<Value> exception = tc.Exception();
		String::Utf8Value e_str(exception);
		ROS_ERROR("Exception: %s", *e_str);
		return false;
	}
	
	v8::String::Utf8Value rs(v);
	result = std::string(*rs, rs.length());

	return true;

}

