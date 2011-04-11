#ifndef ENGINE_V8_HPP
#define ENGINE_V8_HPP

#include "script_engine/EvalScript.h"
#include "script_engine/CompileScript.h"
#include "script_engine/RunScript.h"
#include <v8.h>
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <string>
#include <tr1/unordered_map>

namespace script_engine
{
	class engine_module;

	/// \brief A V8-based JS execution engine.
	///
	/// Registers a few ROS services.
	class engine_v8
	{
	public:
		engine_v8();
		~engine_v8();

		/// \brief Compile and run the given script.
		///
		/// \param src The script's source.
		/// \param result A reference to the result.
		/// \returns False if an error occurs.
		bool eval(const std::string& src, std::string& result);

		/// \brief Compile and register a script
		bool compile(const std::string& name, const std::string& src);

		/// \brief Run a precompiled and registered script.
		bool run(const std::string& name, std::string& result);

	private:
		bool eval_srv(EvalScript::Request&, EvalScript::Response&);
		bool compile_srv(CompileScript::Request&, CompileScript::Response&);
		bool run_srv(RunScript::Request&, RunScript::Response&);
		bool run_script(const v8::Handle<v8::Script>& s, std::string& result);

		v8::HandleScope scope_;
		v8::Handle<v8::ObjectTemplate> global_;
		v8::Handle<v8::Context> context_;

		typedef std::vector<engine_module*> modules_list_t;
		modules_list_t modules_list_;

		ros::ServiceServer srv_eval_script_;
		ros::ServiceServer srv_compile_script_;
		ros::ServiceServer srv_run_script_;

		typedef pluginlib::ClassLoader<engine_module> module_loader_t;
		typedef boost::shared_ptr<module_loader_t> module_loader_ptr_t;
		module_loader_ptr_t module_loader_;

		typedef v8::Handle<v8::Script> script_t;
		typedef std::tr1::unordered_map<std::string, script_t> scripts_map_t;
		scripts_map_t scripts_map_;

	};

}

#endif

