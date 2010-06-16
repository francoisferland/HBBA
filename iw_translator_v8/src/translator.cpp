#include "iw_translator_v8/ExecScript.h"
#include "iw_translator_v8/AddStrategy.h"
#include "iw_translator_v8/SetResourceMax.h"
#include <iw/DesiresSet.h>
#include <topic_filters_manager/manager.hpp>
#include <topic_filters/SetState.h>
#include <iw_solver_gecode/iw_solver_gecode.hpp>
#include <ros/ros.h>
#include <v8.h>
#include <map>
#include <boost/shared_ptr.hpp>

namespace iw_translator_v8
{
	v8::Handle<v8::Value> js_print(const v8::Arguments& args)
	{
		for (int i = 0; i < args.Length(); ++i)
		{
			v8::HandleScope handle_scope;
			v8::String::Utf8Value str(args[i]);
			const char* cstr = *str;
			ROS_INFO("js: %s", cstr);
		}

		return v8::Undefined();

	}

	template <class SolverT>
	class translator
	{
	public:
		translator(); 

	private:
		typedef v8::Handle<v8::Script> script_t;

		void compile_script(const std::string& name, const std::string& src);
		void run_script(const std::string& name);
		void run_script(const script_t& script);

		void exec_script(const std::string& src);
		bool exec_script_srv(ExecScript::Request& req, 
			ExecScript::Response& res);

		bool add_strategy_srv(AddStrategy::Request& req,
			AddStrategy::Response& res);

		bool set_resource_max_srv(SetResourceMax::Request& req,
			SetResourceMax::Response& res);

		void desires_cb(const iw::DesiresSet::ConstPtr& msg);

		static v8::Handle<v8::Value> js_activate(const v8::Arguments& args);
		static v8::Handle<v8::Value> js_deactivate(const v8::Arguments& args);

		ros::NodeHandle n_;
		ros::ServiceServer srv_exec_;
		ros::ServiceServer srv_add_strat_;
		ros::ServiceServer srv_set_res_max_;
		ros::Subscriber sub_desires_;

		v8::HandleScope scope_;
		v8::Handle<v8::ObjectTemplate> global_;
		v8::Handle<v8::Context> context_;

		typedef topic_filters_manager::manager manager_t;
		typedef boost::shared_ptr<manager_t> manager_ptr_t;
		static manager_ptr_t tfm_;

		typedef std::map<std::string, script_t> scripts_map_t;
		scripts_map_t scripts_;

		// The first script in the pair is the bringup, the second the
		// bringdown.
		typedef std::map< std::string, std::pair<script_t, script_t> > 
			strat_map_t;
		strat_map_t strats_;

		typedef typename SolverT::base_t solver_base_t;
		typedef boost::shared_ptr<solver_base_t> solver_base_ptr_t;
		solver_base_ptr_t solver_;

	};

	template <class T>
	typename translator<T>::manager_ptr_t translator<T>::tfm_;

	template <class SolverT>
	translator<SolverT>::translator():
		global_(v8::ObjectTemplate::New()),
		solver_(new SolverT())
	{
		if (!tfm_)
			tfm_.reset(new manager_t());

		global_->Set(v8::String::New("print"),
			v8::FunctionTemplate::New(js_print));

		global_->Set(v8::String::New("activate"), 
			v8::FunctionTemplate::New(&translator::js_activate));
		global_->Set(v8::String::New("deactivate"), 
			v8::FunctionTemplate::New(&translator::js_deactivate));

		context_ = v8::Context::New(NULL, global_);

		srv_exec_ = n_.advertiseService("/exec_script", 
			&translator::exec_script_srv, this);
		srv_add_strat_ = n_.advertiseService("/add_strategy",
			&translator::add_strategy_srv, this);
		srv_set_res_max_ = n_.advertiseService("/set_resource_max",
			&translator::set_resource_max_srv, this);
		sub_desires_ = n_.subscribe("/desires_set", 10,
			&translator::desires_cb, this);

	}

	template <class T>
	void translator<T>::compile_script(const std::string& name, 
		const std::string& src)
	{
		v8::Context::Scope context_scope(context_);
		v8::Handle<v8::String> s = v8::String::New(src.c_str());
		v8::Handle<v8::String> n = v8::String::New(name.c_str());
		scripts_[name] = v8::Script::Compile(s, n);
	}

	template <class T>
	void translator<T>::run_script(const std::string& name)
	{
		run_script(scripts_[name]);
	}

	template <class T>
	void translator<T>::run_script(const script_t& script)
	{
		v8::Context::Scope context_scope(context_);
		script->Run();
	}

	template <class T>
	void translator<T>::exec_script(const std::string& src)
	{
		v8::Context::Scope context_scope(context_);
		v8::Handle<v8::String> s = v8::String::New(src.c_str());
		v8::Handle<v8::Script> script = v8::Script::Compile(s);
		script->Run();
	}

	template <class T>
	bool translator<T>::exec_script_srv(ExecScript::Request& req, 
		ExecScript::Response& res)
	{
		exec_script(req.source);
		return true;
	}

	template <class T>
	bool translator<T>::add_strategy_srv(AddStrategy::Request& req,
		AddStrategy::Response& res)
	{
		// Compile scripts and save the handles.
		std::string bup_name = req.strategy.id + "_bringup";
		std::string bdn_name = req.strategy.id + "_bringdown";
		compile_script(bup_name, req.strategy.bringup);
		compile_script(bdn_name, req.strategy.bringdown);

		strats_[req.strategy.id] = std::make_pair(
			scripts_[bup_name],
			scripts_[bdn_name]);

		// Convert for the solver.
		typename solver_base_t::strat_vec_t c(req.strategy.cost.size());
		for (size_t i = 0; i < c.size(); ++i)
			c[i] = std::make_pair(req.strategy.cost[i].id,
					req.strategy.cost[i].value);
		typename solver_base_t::strat_vec_t u(req.strategy.utility.size());
		for (size_t i = 0; i < c.size(); ++i)
			u[i] = std::make_pair(req.strategy.utility[i].id,
					req.strategy.utility[i].value);
		solver_->add_strategy(req.strategy.id, c, u);

		return true;
	}

	template <class T>
	bool translator<T>::set_resource_max_srv(SetResourceMax::Request& req,
		SetResourceMax::Response& res)
	{
		solver_->set_resource_max(req.id, req.value);
		return true;
	}


	template <class T>
	void translator<T>::desires_cb(const iw::DesiresSet::ConstPtr& msg)
	{
		// Build the utility vector
		typedef std::vector<iw_msgs::Desire> u_vec_t;
		const u_vec_t& desires = msg->desires;
		typename u_vec_t::const_iterator d;
		for (d = desires.begin(); d != desires.end(); ++d)
			solver_->set_info_min(d->type, d->utility);

		// Solve and execute the solution.
		typedef typename solver_base_t::sol_vec_t s_vec_t;
		s_vec_t res(strats_.size());
		solver_->solve(res);
		typename s_vec_t::const_iterator s;
		for (s = res.begin(); s != res.end(); ++s)
			// Reminder: s->first: strat_id, s->second: activation
			// strats_[strat_id].first: bringup, 
			// strats_[strat_id].second: bringdown
			if (s->second) 
				run_script(strats_[s->first].first);
			else
				run_script(strats_[s->first].second);

	}

	template <class T>
	v8::Handle<v8::Value> translator<T>::js_activate(const v8::Arguments& args)
	{
		v8::String::Utf8Value nv(args[0]);
		const char* name = *nv;
		ROS_INFO("Activating %s ...", name);
		topic_filters::SetState req;
		req.request.state = true;
		tfm_->call_filter<topic_filters::SetState>(name, req);
		return v8::Undefined();
	}

	template <class T>
	v8::Handle<v8::Value> 
		translator<T>::js_deactivate(const v8::Arguments& args)
	{
		v8::String::Utf8Value nv(args[0]);
		const char* name = *nv;
		ROS_INFO("Deactivating %s ...", name);
		topic_filters::SetState req;
		req.request.state = false;
		tfm_->call_filter<topic_filters::SetState>(name, req);
		return v8::Undefined();
	}

}

int main(int argc, char** argv)
{
	using namespace iw_translator_v8;
	ros::init(argc, argv, "translator");

	typedef iw_solver_gecode::gecode_solver solver_t;
	translator<solver_t> t;
	
	ros::spin();
}

