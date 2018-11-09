#ifndef IW_TRANSLATOR_HPP
#define IW_TRANSLATOR_HPP

#include <hbba_msgs/Strategy.h>
#include <hbba_msgs/DesiresSet.h>
#include <hbba_msgs/Intention.h>
#include <iw_translator/solver_model.hpp>
#include <iw_translator/solver.hpp>
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include <script_engine/engine_v8.hpp>

namespace iw_translator
{
    /// \brief An integrated solver and strategy activator for the Intention
    /// Workspace.
    /// 
    /// Based on a static model of the capacities of the robot (strategies and
    /// resources caps), the integrated translator produces the intention of the
    /// robot from the dynamic active desires set.
    /// The intention is a strategy activation vector (a) which is used to
    /// produce a single activation/deactivation script for the V8-based script
    /// engine.
    ///
    /// Topics:
    ///  - desires_set: The current, active desires set produced by the IW.
    ///  - intention:   The solved intention for the desire set.
    ///  - iw_status:   Compact form of the IW Translator status.
    ///
    /// Services:
    ///  - See script_engine for V8 JavaScript services.
    ///  
    /// Parameters (note that ~ points to the np namespace given to the
    //  constructor):
    ///  - ~strategies: An array of strategy definitions.
    ///                 Mandatory parameter - if it is not available, the 
    ///                 translator will not be initialized and will not run.
    ///                 See StrategyParser for details.
    ///  - ~res_caps:   An array of resource capacity definitions.
    ///  - ~max_p:      Maximise total utility production (p).
    ///                 Default: true.
    ///  - ~solver_sa:  Perform simulated annealing.
    ///                 Default: false.
    ///  - ~solver_log: Solver search log in standard output.
    ///                 Default: false.
    ///  - ~time_limit: Solver search time limit, in ms.
    ///                 Default: 0 (no limit).
    ///
    class IWTranslator
    {
    private:
        std::vector<hbba_msgs::Strategy> strats_;
        boost::scoped_ptr<SolverModel>   solver_model_;
        script_engine::engine_v8         script_engine_;

        ros::Subscriber                  sub_desires_;
        ros::Publisher                   pub_intention_;
        ros::Publisher                   pub_status_;
        ros::ServiceClient               srv_eval_script_;

        SolverParams                     solver_params_;

        std::vector<unsigned char>       last_a_;
        std::vector<std::string>         last_p_;

    public:
        /// \brief Constructor.
        ///
        /// \param n  Node handle for topics and services.
        /// \param np Node handle for parameters.
        ///
        IWTranslator(ros::NodeHandle& n, ros::NodeHandle& np);

    private:
        void desiresCB(const hbba_msgs::DesiresSet::ConstPtr& msg);

        /// \brief Evaluate the source script of each strategies, warn the user
        /// in case of errors.
        void initStrategies();

        /// \brief Evaluate the specified script, warn the user in case of errors.
        void evalScript(const std::string& script);

        /// \brief Call activation (or deactivation) scripts for each strategy.
        ///
        /// Only call scripts on activation state and parameter changes.
        /// NOTE: This function assumes the strategies in intent are ordered as
        /// in the strats_ vector.
        void activateIntention(const hbba_msgs::Intention& intent);

    };
}

#endif

