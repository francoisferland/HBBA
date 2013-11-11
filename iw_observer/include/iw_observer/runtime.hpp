#ifndef RUNTIME_HPP
#define RUNTIME_HPP

#include <hbba_msgs/Event.h>
#include <hbba_msgs/AddDesires.h>
#include <hbba_msgs/RemoveDesires.h>
#include <ros/ros.h>
#include "rules_ast.hpp"

namespace iw_observer
{
    /// \brief Runtime for the IWObserver rules generator.
    ///
    /// Provides a simple interface to the IW services.
    /// 
    /// Topics (input):
    ///  - events         (hbba_msgs/Event).
    ///
    /// Services (as a client):
    ///  - add_desires    (hbba_msgs/AddDesires).
    ///  - remove_desires (hbba_msgs/RemoveDesires).
    /// 
    class Runtime
    {
    private:
        ros::Subscriber    sub_events_;
        ros::ServiceClient scl_add_;
        ros::ServiceClient scl_del_;

        Rules rules_;

    public:
        /// \brief Constructor.
        ///
        /// Create service proxies and assumes they are in the node's namespace.
        ///
        /// \param rules The parsed ruleset to observe.
        Runtime(const Rules& rules);

        /// \brief Add a desire to the Intention Workspace.
        void addDesire(const hbba_msgs::Desire& d);

        /// \brief Delete a desire from the Intention Workspace.
        void removeDesire(const std::string& id);

    private:
        void eventsCB(const hbba_msgs::Event::ConstPtr& msg);

    };
}

#endif

