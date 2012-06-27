#include "iw_solver_base.hpp"

#include <hbba_msgs/SetResourceMax.h>
#include <hbba_msgs/AddStrategy.h>
#include <hbba_msgs/DesiresSet.h>
#include <hbba_msgs/ResourcesSet.h>
#include <hbba_msgs/Intention.h>
#include <script_engine/EvalScript.h>
#include <std_msgs/Duration.h>
#include <ros/ros.h>
#include <algorithm>

namespace iw_solver_interface
{
    /// \brief Default scalar type for values (cost, intensity, ...).
    ///
    /// Should fit with the declared ROS message type(s).
    typedef double DefaultScalar;

    /// \brief A base class for IW solver nodes.
    ///
    /// You need to provide it with your low-level solver class, for instance
    /// an or-tools's constraint_solver wrapper (see iw_solver_base for 
    /// details), and this class will provide services and topics for your
    /// HBBA instance.
    ///
    /// RAII design, could be used in a nodelet context if you create the
    /// object at the right time.
    template <class T, class S = DefaultScalar>
    class SolverNode: public iw_solver_base<T, S>
    {
    public:
        typedef S Scalar;
        typedef iw_solver_base<T, S> BaseType;
        typedef SolverNode<T, S> ThisType;

        SolverNode(
            const boost::shared_ptr<T>& impl = boost::shared_ptr<T>(new T()),
            const ros::NodeHandle& n = ros::NodeHandle()): 
            BaseType(impl), n_(n)
        {
            srv_add_strat_ = n_.advertiseService("add_strategy",
                &ThisType::addStrategySrv, this);
            srv_set_res_max_ = n_.advertiseService("set_resource_max",
                &ThisType::setResourceMaxSrv, this);
            sub_desires_ = n_.subscribe("desires_set", 10,
                &ThisType::desiresCB, this);

            pub_intention_ = 
                n_.advertise<hbba_msgs::Intention>("intention", 10);
            pub_res_max_ =
                n_.advertise<hbba_msgs::ResourcesSet>("resource_max", 10, true);
            ROS_INFO("Waiting for script_engine service(s) ...");
            ros::service::waitForService("eval_script");
            scl_eval_script_ = 
               n_.serviceClient<script_engine::EvalScript>("eval_script", true);

            ros::NodeHandle np("~");
            pub_solve_time_ =
                np.advertise<std_msgs::Duration>("solve_time", 1);
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
            typename BaseType::strat_vec_t c(req.strategy.cost.size());
            for (size_t i = 0; i < c.size(); ++i)
                c[i] = std::make_pair(strat.cost[i].id, strat.cost[i].value);
            typename BaseType::strat_vec_t u(1);
            u[0] = std::make_pair(strat.utility.id, strat.utility.value);
            typename BaseType::strat_vec_t 
                u_min(req.strategy.utility_min.size());
            for (size_t i = 0; i < u_min.size(); ++i)
                u_min[i] = std::make_pair(strat.utility_min[i].id, 
                    strat.utility_min[i].value);

            BaseType::add_strategy(strat.id, c, u, u_min);
            return true;
        }

        bool setResourceMaxSrv(hbba_msgs::SetResourceMax::Request& req,
            hbba_msgs::SetResourceMax::Response&)
        {
            BaseType::set_resource_max(req.id, req.value);
            resource_max_[req.id] = req.value;
            publishResMax();
            return true;
        }

        void desiresCB(const hbba_msgs::DesiresSet::ConstPtr& msg)
        {
            typename BaseType::sol_vec_t result(BaseType::strat_count());

            // Do not solve empty desire sets for now.
            if (msg->desires.size() < 1) 
            {
                // Set every strategies to disabled first.
                strats_map_t::const_iterator i;
                int j = 0;
                for (i = strategies_.begin(); i != strategies_.end(); ++i, ++j)
                    result[j] = std::make_pair(i->first, false);
                applyStrats(result, msg->desires);

                return;
            }

            ros::Time start_time = ros::Time::now();

            // Build the utility requests vector.
            const u_vec_t& desires = msg->desires;
            typedef typename u_vec_t::const_iterator u_vec_it;
            BaseType::clear_reqs();
            for (u_vec_it i = desires.begin(); i != desires.end(); ++i)
            {
                BaseType::set_util_min(i->type, i->utility);
                BaseType::set_util_int(i->type, i->intensity);
            }

            // Automatically call the solver.
            BaseType::solve(result);
            ROS_DEBUG("Solving done.");

            applyStrats(result, desires);

            std_msgs::Duration solve_time;
            solve_time.data = ros::Time::now() - start_time;
            pub_solve_time_.publish(solve_time);
        }


    private:
        typedef std::vector<hbba_msgs::Desire> u_vec_t;

        /// \brief Apply strategies according to a result set.
        void applyStrats(typename BaseType::sol_vec_t& result, 
            const u_vec_t& desires)
        {
            // Sort the results on activation value so that bringdown scripts
            // are called before bringup ones.
            // Important for mutually exclusive strategies.
            std::sort(result.begin(), result.end(),
                boost::bind(&BaseType::sol_t::second, _1) <
                boost::bind(&BaseType::sol_t::second, _2));

            // Publish the intention set for diagnostic purposes
            // Note that desire fields can be empty, since not every single 
            // strategy can directly be mapped to a desire.
            // We only keep direct associations.
            hbba_msgs::Intention intent;
            intent.strategies.reserve(result.size());
            intent.desires.resize(result.size());
            intent.enabled.resize(result.size());

            typedef typename BaseType::sol_vec_t::const_iterator CI;
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
                        intent.desire_types[j] = d->type;
                    }
                }   
                script += ");";

                // Run the corresponding script.
                script_engine::EvalScript eval;
                eval.request.source = script;
                //ROS_INFO("Function call: %s", script.c_str());
                scl_eval_script_.call(eval);
            }
	    
	    // Stamp the message
	    intent.stamp = ros::Time::now();

            pub_intention_.publish(intent);
        }

        void publishResMax()
        {
            hbba_msgs::ResourcesSet msg;
            msg.set.reserve(resource_max_.size());
            typename std::map<std::string, Scalar>::const_iterator i;
            for (i = resource_max_.begin(); i != resource_max_.end(); ++i)
            {
                hbba_msgs::ResourceUsage u; 
                u.id = i->first;
                u.value = i->second;
                msg.set.push_back(u);
            }
            pub_res_max_.publish(msg);
        }

        typedef std::map<std::string, hbba_msgs::Strategy> strats_map_t;
        strats_map_t strategies_;
        std::map<std::string, Scalar> resource_max_;

        ros::NodeHandle n_;
        ros::ServiceServer srv_add_strat_;
        ros::ServiceServer srv_set_res_max_;
        ros::ServiceClient scl_eval_script_;
        
        ros::Subscriber sub_desires_;
        ros::Publisher pub_intention_;
        ros::Publisher pub_res_max_;
        ros::Publisher pub_solve_time_;

    };

}
