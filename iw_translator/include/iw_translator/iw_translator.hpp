#ifndef IW_TRANSLATOR_HPP
#define IW_TRANSLATOR_HPP

#include <hbba_msgs/Strategy.h>
#include <hbba_msgs/DesiresSet.h>
#include <iw_translator/solver_model.hpp>
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>

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
    ///  
    /// Parameters (note that ~ points to the np namespace given to the
    //  constructor):
    ///  - ~strategies: An array of strategy definitions.
    ///                 Mandatory parameter - if it is not available, the 
    ///                 translator will not be initialized and will not run.
    ///                 See StrategyParser for details.
    ///  - ~res_caps:   An array of resource capacity definitions.
    ///
    class IWTranslator
    {
    private:
        std::vector<hbba_msgs::Strategy> strats_;
        boost::scoped_ptr<SolverModel>   solver_model_;

        ros::Subscriber                  sub_desires_;

    public:
        /// \brief Constructor.
        ///
        /// \param n  Node handle for topics and services.
        /// \param np Node handle for parameters.
        ///
        IWTranslator(ros::NodeHandle& n, ros::NodeHandle& np);

    private:
        void desiresCB(const hbba_msgs::DesiresSet::ConstPtr& msg);

    };
}

#endif

