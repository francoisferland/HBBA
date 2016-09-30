#include <script_engine/engine_v8.hpp>
#include <script_engine_plugins/engine_module.hpp>

using namespace script_engine;

namespace {
	v8::Handle<v8::Value> se_log(const v8::Arguments& args)
	{
		v8::String::Utf8Value v(args[0]);
		const char* val = *v;
		ROS_INFO("se_log: %s", val);
		return v8::True();
	}

	v8::Handle<v8::Value> se_error(const v8::Arguments& args)
	{
		v8::String::Utf8Value v(args[0]);
		const char* val = *v;
		ROS_ERROR("se_error: %s", val);
		return v8::True();
	}
}

engine_v8::engine_v8():
//	global_(v8::ObjectTemplate::New()),
//	context_(v8::Context::New(NULL, global_)),
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

	global_ = v8::ObjectTemplate::New();
		
	// Log functions.
	global_->Set(v8::String::New("se_log"),
		v8::FunctionTemplate::New(se_log));
	global_->Set(v8::String::New("se_error"),
		v8::FunctionTemplate::New(se_error));

	// Load plugins.
	std::vector<std::string> classes = module_loader_->getDeclaredClasses();
	std::vector<std::string>::const_iterator i;
	for (i = classes.begin(); i != classes.end(); ++i)
	{
		try 
		{
            ROS_INFO("Loading script engine plugin %s...", i->c_str());
			engine_module* m = module_loader_->createUnmanagedInstance(*i);
			m->init(global_);
			modules_list_.push_back(m);
		}
		catch (pluginlib::LibraryLoadException e)
		{
			ROS_ERROR("Cannot load script_engine plugin, exception: \n %s", 
				e.what());
		}
	}

	context_ = v8::Context::New(NULL, global_);
	
}

engine_v8::~engine_v8()
{
	modules_list_t::iterator i;
	for (i = modules_list_.begin(); i != modules_list_.end(); ++i)
		delete *i;
}

bool engine_v8::eval(const std::string& src, std::string& result)
{
	ROS_DEBUG("eval(\"%s\") ...", src.c_str());
	using namespace v8;
	Context::Scope context_scope(context_);
	Handle<String> str = String::New(src.c_str());

    Handle<Script> script;
    {
        TryCatch tc;
        script = Script::Compile(str);
        if (tc.HasCaught())
        {
            result = *String::AsciiValue(tc.Message()->Get());
            ROS_ERROR("V8 compile error: %s: %s \n when running: \n\t%s",
                result.c_str(),
                *String::AsciiValue(tc.Message()->GetSourceLine()),
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
	scripts_map_[name] = Script::Compile(String::New(src.c_str()));

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
		String::AsciiValue e_str(exception);
		ROS_ERROR("Exception: %s", *e_str);
		return false;
	}
	
	v8::String::Utf8Value rs(v);
	result = std::string(*rs, rs.length());

	return true;

}
