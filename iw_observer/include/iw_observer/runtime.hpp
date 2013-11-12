#ifndef RUNTIME_HPP
#define RUNTIME_HPP

#include "rules_ast.hpp"
#include <hbba_msgs/Event.h>
#include <hbba_msgs/UpdateDesires.h>
#include <ros/ros.h>
#include <map>

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
    ///  - update_desires (hbba_msgs/UpdateDesires).
    /// 
    class Runtime
    {
    private:
        ros::Subscriber    sub_events_;
        ros::ServiceClient scl_update_;

        const Rules& rules_;

        typedef std::map<std::string, uint8_t> EventTypeMap; 
        /// \brief Maps an event string (e.g. "exp_on") to an enum value in
        /// hbba_msgs::Event.
        /// The map is filled at the first construction of a Runtime instance.
        static EventTypeMap event_type_map_; 

        typedef std::map<std::string, Commands> CmdsMap;
        /// \brief Maps a desire type to a set of commands to execute.
        ///
        /// Used by FiltersMap.
        CmdsMap cmds_map_;

        typedef std::map<int, CmdsMap> FiltersMap;
        /// \brief Maps an event type id (int) to a map of rules to evaluate.
        FiltersMap filters_map_;

        // The following request object contain desires to add and remove when 
        // commands are executed.
        // Since a single event can produce more than one call to the IW, we
        // cache these when rules are evaluated, and then we used to service
        // proxies.
        // The vectors are flushed after being used in eventsCB.
        hbba_msgs::UpdateDesires update_des_; 

    public:
        /// \brief Constructor.
        ///
        /// Create service proxies and assumes they are in the node's namespace.
        ///
        /// \param rules The parsed ruleset to observe.
        ///              NOTE: Keeps pointers to members owned by the ruleset,
        ///              and thus needs to be kept in memory for the lifetime of
        ///              the runtime. 
        Runtime(const Rules& rules);

        /// \brief Add a desire to the Intention Workspace.
        void addDesire(const hbba_msgs::Desire& d);

        /// \brief Delete a desire from the Intention Workspace.
        void removeDesire(const std::string& id);

    private:
        void eventsCB(const hbba_msgs::Event::ConstPtr& msg);
        void parseRules(const Rules& rules);

    };
}

#endif

