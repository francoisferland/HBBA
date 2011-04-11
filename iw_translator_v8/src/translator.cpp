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

		typedef iw_translator_v8::Strategy strat_t;
		typedef std::map<std::string, strat_t> 
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
		// TODO: Replace following code with an evaluation of the source of the
		// strategy, which will register bringup and bringdown functions right
		// into the context. Save the name specified in the strategy pair,
		//std::string bup_name = req.strategy.id + "_bringup";
		//std::string bdn_name = req.strategy.id + "_bringdown";
		//engine_.compile(bup_name, req.strategy.bringup);
		//engine_.compile(bdn_name, req.strategy.bringdown);

		std::string r;
		if (!engine_.eval(req.strategy.source, r))
		{
			ROS_ERROR("An error occured while evaluating %s's source: %s",
				req.strategy.id.c_str(), r.c_str());
			return false;
		}

		// Save a full copy.
		strats_[req.strategy.id] = req.strategy;

		// Convert for the solver.
		typename solver_base_t::strat_vec_t c(req.strategy.cost.size());
		for (size_t i = 0; i < c.size(); ++i)
			c[i] = std::make_pair(req.strategy.cost[i].id,
					req.strategy.cost[i].value);
		// TODO: Replace the utility vector with a single declaration in the
		// solver.
		//typename solver_base_t::strat_vec_t u(req.strategy.utility.size());
		//for (size_t i = 0; i < u.size(); ++i)
		//	u[i] = std::make_pair(req.strategy.utility[i].id,
		//			req.strategy.utility[i].value);
		typename solver_base_t::strat_vec_t u(1);
		u[0] = std::make_pair(req.strategy.utility.id,
			req.strategy.utility.value);
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
		// Desire type -> params map. Note that the incoming desires' set 
		// shouldn't contain more than one instance of a single utility type, 
		// e.g., there shouldn't be two "goto" desires.
		typedef std::map<std::string, std::string> type_params_map_t;
		type_params_map_t type_params;

		// Build the utility vector and type map
		typedef std::vector<iw_msgs::Desire> u_vec_t;
		const u_vec_t& desires = msg->desires;
		typename u_vec_t::const_iterator d;
		for (d = desires.begin(); d != desires.end(); ++d)
		{
			solver_->set_info_min(d->type, d->utility);
			type_params[d->type] = d->params;
		}

		// Solve and execute the solution.
		typedef typename solver_base_t::sol_vec_t s_vec_t;
		s_vec_t res(strats_.size());
		solver_->solve(res);
		typename s_vec_t::const_iterator s;
		std::string result; // Ignored for now.
		for (s = res.begin(); s != res.end(); ++s)
		{
			std::string script;
			const strat_t& strat = strats_[s->first];
			if (s->second) 
				script = strat.bringup_function;
			else
				script = strat.bringdown_function;

			script += "(" + type_params[strat.utility.id] + ");";
			engine_.eval(script, result);
		}
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

