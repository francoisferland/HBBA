#include "iw_solver_base.hpp"

#include <hbba_msgs/SetResourceMax.h>
#include <hbba_msgs/AddStrategy.h>
#include <hbba_msgs/DesiresSet.h>
#include <hbba_msgs/ResourcesSet.h>
#include <hbba_msgs/Intention.h>
#include <script_engine/EvalScript.h>
#include <ros/ros.h>

namespace iw_solver_interface
{
    /// \brief A base class for IW solver nodes.
    ///
    /// You need to provide it with your low-level solver class, for instance
    /// an or-tools's constraint_solver wrapper (see iw_solver_base for 
    /// details), and this class will provide services and topics for your
    /// HBBA instance.
    ///
    /// RAII design, could be used in a nodelet context if you create the
    /// object at the right time.
    template <class T>
    class SolverNode: public iw_solver_base<T>
    {
    public:
        SolverNode(const ros::NodeHandle& n = ros::NodeHandle()): n_(n)
        {
            srv_add_strat_ = n_.advertiseService("add_strategy",
                &this_t::addStrategySrv, this);
            srv_set_res_max_ = n_.advertiseService("set_resource_max",
                &this_t::setResourceMaxSrv, this);
            sub_desires_ = n_.subscribe("desires_set", 10,
                &this_t::desiresCB, this);

            pub_intention_ = 
                n_.advertise<hbba_msgs::Intention>("intention", 10);
            pub_res_max_ =
                n_.advertise<hbba_msgs::ResourcesSet>("resource_max", 10, true);
            ROS_INFO("Waiting for script_engine service(s) ...");
            ros::service::waitForService("eval_script");
            scl_eval_script_ = 
               n_.serviceClient<script_engine::EvalScript>("eval_script", true);
        }

        bool addStrategySrv(hbba_msgs::AddStrategy::Request& req,
            hbba_msgs::AddStrategy::Response&)
        {
            script_engine::EvalScript eval;
            eval.request.source =  req.strategy.source;
            if (!scl_eval_script_.call(eval))
            {
                ROS_ERROR("An error occured while evaluating %s's source: %s",
                    req.strategy.id.c_str(), eval.response.result.c_str());
                return false;
            }

            // TODO: We might want to check if the bringup/bringdown scripts
            // are valid, but we don't want to tie the solver to any script
            // engine.
            const hbba_msgs::Strategy& strat = req.strategy;
            // We're saving a full copy of the strategy for now.
            // TODO: Give access to the strategies database for client classes.
            strategies_[strat.id] = strat;

            // Convert the cost, utility and utility needed vectors
            typename base_t::strat_vec_t c(req.strategy.cost.size());
            for (size_t i = 0; i < c.size(); ++i)
                c[i] = std::make_pair(strat.cost[i].id, strat.cost[i].value);
            typename base_t::strat_vec_t u(1);
            u[0] = std::make_pair(strat.utility.id, strat.utility.value);
            typename base_t::strat_vec_t u_min(req.strategy.utility_min.size());
            for (size_t i = 0; i < u_min.size(); ++i)
                u_min[i] = std::make_pair(strat.utility_min[i].id, 
                    strat.utility_min[i].value);

            base_t::add_strategy(strat.id, c, u, u_min);
            return true;
        }

        bool setResourceMaxSrv(hbba_msgs::SetResourceMax::Request& req,
            hbba_msgs::SetResourceMax::Response&)
        {
            base_t::set_resource_max(req.id, req.value);
            resource_max_[req.id] = req.value;
            publishResMax();
            return true;
        }

        void desiresCB(const hbba_msgs::DesiresSet::ConstPtr& msg)
        {
            // Do not solve empty desire sets for now.
            if (msg->desires.size() < 1) 
                return;

            // Build the utility requests vector.
            typedef std::vector<hbba_msgs::Desire> u_vec_t;
            const u_vec_t& desires = msg->desires;
            typedef typename u_vec_t::const_iterator u_vec_it;
            base_t::clear_reqs();
            for (u_vec_it i = desires.begin(); i != desires.end(); ++i)
                base_t::set_util_min(i->type, i->utility);

            // Automatically call the solver.
            typename base_t::sol_vec_t result(base_t::strat_count());
            base_t::solve(result);

            // Publish the intention set for diagnostic purposes
            // Note that desire fields can be empty, since not every single 
            // strategy can directly be mapped to a desire.
            // We only keep direct associations.
            hbba_msgs::Intention intent;
            intent.strategies.reserve(result.size());
            intent.desires.reserve(result.size());
            intent.enabled.resize(result.size()); 

            typedef typename base_t::sol_vec_t::const_iterator CI;
            size_t j = 0; // Used to map desires to strategies.
            for (CI i = result.begin(); i != result.end(); ++i, ++j)
            {
                // Add strategy to the intention message.
                intent.strategies.push_back(i->first);
                intent.enabled.push_back(i->second);

                // Get the name of the script function.
                const hbba_msgs::Strategy& s = strategies_[i->first];
                std::string script;
                if (i->second)
                    script = s.bringup_function;
                else
                    script = s.bringdown_function;

                // Map desires params to strategies.
                script += "(";
                typedef typename u_vec_t::const_iterator u_vec_it;
                for (u_vec_it d = desires.begin(); d != desires.end(); ++d) 
                {   
                    if(s.utility.id == d->type)
                    {
                        script += d->params;
                        intent.desires[j] = d->id;
                    }
                }   
                script += ");";

                // Run the corresponding script.
                script_engine::EvalScript eval;
                eval.request.source = script;
                ROS_INFO("Function call: %s", script.c_str());
                scl_eval_script_.call(eval);
            }

            pub_intention_.publish(intent);
            
        }

    private:
        typedef iw_solver_base<T> base_t;
        typedef SolverNode<T> this_t;

        void publishResMax()
        {
            hbba_msgs::ResourcesSet msg;
            msg.set.reserve(resource_max_.size());
            std::map<std::string, int>::const_iterator i;
            for (i = resource_max_.begin(); i != resource_max_.end(); ++i)
            {
                hbba_msgs::ResourceUsage u; 
                u.id = i->first;
                u.value = i->second;
                msg.set.push_back(u);
            }
            pub_res_max_.publish(msg);
        }

        std::map<std::string, hbba_msgs::Strategy> strategies_;
        std::map<std::string, int> resource_max_;

        ros::NodeHandle n_;
        ros::ServiceServer srv_add_strat_;
        ros::ServiceServer srv_set_res_max_;
        ros::ServiceClient scl_eval_script_;
        
        ros::Subscriber sub_desires_;
        ros::Publisher pub_intention_;
        ros::Publisher pub_res_max_;


    };

}
