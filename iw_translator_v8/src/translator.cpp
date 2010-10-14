#include "iw_translator_v8/AddStrategy.h"
#include "iw_translator_v8/SetResourceMax.h"
#include <iw_msgs/DesiresSet.h>
#include <topic_filters_manager/manager.hpp>
#include <topic_filters/SetState.h>
#include <iw_solver_gecode/iw_solver_gecode.hpp>
#include <script_engine/engine_v8.hpp>
#include <ros/ros.h>
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
		typedef std::string script_t;

		bool add_strategy_srv(AddStrategy::Request& req,
			AddStrategy::Response& res);

		bool set_resource_max_srv(SetResourceMax::Request& req,
			SetResourceMax::Response& res);

		void desires_cb(const iw_msgs::DesiresSet::ConstPtr& msg);

		static v8::Handle<v8::Value> js_activate(const v8::Arguments& args);
		static v8::Handle<v8::Value> js_deactivate(const v8::Arguments& args);

		ros::NodeHandle n_;
		ros::ServiceServer srv_add_strat_;
		ros::ServiceServer srv_set_res_max_;
		ros::Subscriber sub_desires_;

		script_engine::engine_v8 engine_;

		// The first script in the pair is the bringup, the second the
		// bringdown.
		typedef std::map< std::string, std::pair<script_t, script_t> > 
			strat_map_t;
		strat_map_t strats_;

		typedef typename SolverT::base_t solver_base_t;
		typedef boost::shared_ptr<solver_base_t> solver_base_ptr_t;
		solver_base_ptr_t solver_;

	};

	template <class SolverT>
	translator<SolverT>::translator():
		solver_(new SolverT())
	{
		srv_add_strat_ = n_.advertiseService("add_strategy",
			&translator::add_strategy_srv, this);
		srv_set_res_max_ = n_.advertiseService("set_resource_max",
			&translator::set_resource_max_srv, this);
		sub_desires_ = n_.subscribe("desires_set", 10,
			&translator::desires_cb, this);

	}

	template <class T>
	bool translator<T>::add_strategy_srv(AddStrategy::Request& req,
		AddStrategy::Response& res)
	{
		// Compile scripts and save the handles.
		std::string bup_name = req.strategy.id + "_bringup";
		std::string bdn_name = req.strategy.id + "_bringdown";
		engine_.compile(bup_name, req.strategy.bringup);
		engine_.compile(bdn_name, req.strategy.bringdown);

		strats_[req.strategy.id] = std::make_pair(
			bup_name,
			bdn_name);

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
	void translator<T>::desires_cb(const iw_msgs::DesiresSet::ConstPtr& msg)
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
				engine_.run(strats_[s->first].first);
			else
				engine_.run(strats_[s->first].second);

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

