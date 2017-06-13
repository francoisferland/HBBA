#include <iw_observer/runtime.hpp>

using namespace iw_observer;

Runtime::EventTypeMap Runtime::event_type_map_;

Runtime::Runtime(const Rules& rules): rules_(rules)
{
    if (event_type_map_.empty()) {
        event_type_map_["des_on"]  = hbba_msgs::Event::DES_ON;
        event_type_map_["des_off"] = hbba_msgs::Event::DES_OFF;
        event_type_map_["int_on"]  = hbba_msgs::Event::INT_ON;
        event_type_map_["int_off"] = hbba_msgs::Event::INT_OFF;
        event_type_map_["exp_on"]  = hbba_msgs::Event::EXP_ON;
        event_type_map_["exp_off"] = hbba_msgs::Event::EXP_OFF;
        event_type_map_["acc_on"]  = hbba_msgs::Event::ACC_ON;
        event_type_map_["acc_off"] = hbba_msgs::Event::ACC_OFF;
    }

    ros::NodeHandle n;

    sub_events_ = n.subscribe("events", 100, &Runtime::eventsCB, this);

    ROS_INFO("Waiting for 'update_desires' service...");
    ros::service::waitForService("update_desires");
    scl_update_ = n.serviceClient<hbba_msgs::UpdateDesires>("update_desires", 
                                                            true);

    parseRules(rules);
}

void Runtime::addDesire(const hbba_msgs::Desire& d)
{
    ROS_DEBUG("IWObserver rt: add desire of type '%s'",
              d.type.c_str());
    update_des_.request.add.push_back(d);

}

void Runtime::removeDesire(const std::string& id)
{
    ROS_DEBUG("IWObserver rt: del desire with id '%s'",
              id.c_str());
    update_des_.request.remove.push_back(id);

}

void Runtime::eventsCB(const hbba_msgs::Event::ConstPtr& msg)
{
    // 1. Look for the event type in the map, then the desire type.
    // 2. Execute the matching commands.
    int et = msg->type;
    FiltersMap::const_iterator fmi = filters_map_.find(et);
    if (fmi == filters_map_.end()) {
        // Here we don't care, the event type just doesn't match any rules.
        return;
    }

    const CmdsMap&     cmds_map = fmi->second;
    const std::string& des_type = msg->desire_type;

    const CmdsMap::const_iterator cmi = cmds_map.find(des_type);
    if (cmi == cmds_map.end()) {
        // Once again, we don't care.
        return;
    }

    const Commands& cmds = cmi->second;
    typedef Commands::const_iterator It;
    for (It i = cmds.begin(); i != cmds.end(); ++i) {
        const Command& cmd = **i;
        cmd.exec(*this, *msg);
    }

    // Look for changes in both add and delete vectors, call the proper
    // services.
    std::vector<hbba_msgs::Desire>& add_set = update_des_.request.add;
    std::vector<std::string>&       rem_set = update_des_.request.remove;
    if (!add_set.empty() || !rem_set.empty()) {
        ROS_DEBUG("add_set size: %lu, rem_set size: %lu",
                  add_set.size(),
                  rem_set.size());
        if (!scl_update_.call(update_des_)) {
            ROS_ERROR("Error on update service call!");
            return;
        }
        ROS_DEBUG("Update call done, will clear the sets for the next round.");
        add_set.clear();
        rem_set.clear();
    } else {
        ROS_DEBUG("Empty desire updates set, ignoring.");
    }
}

void Runtime::parseRules(const Rules& rules)
{
    typedef Rules::const_iterator It;
    for (It i = rules.begin(); i != rules.end(); ++i) {
        // 1. Parse the event type
        // 2. Look for/Add the class(es) to the event type entry.
        // 3. Add the commands to the class(es) entry.
        //    The maps structure is event_type -> desire_classes -> vectors of
        //    pointers to commands.
       
        Rule* rule = *i;
        const Filter&          filter = rule->filter();
        const std::string&     ets    = filter.eventType();
        EventTypeMap::iterator eti = event_type_map_.find(ets);  
        if (eti == event_type_map_.end()) {
            ROS_WARN("Unknown event type '%s', skipping rule.", 
                     ets.c_str());
            continue;
        }

        int et = eti->second; // Event type in enum.

        // Append the rule's command pointers per desire type.
        CmdsMap&        cmds_map  = filters_map_[et];
        const Commands& rule_cmds = rule->commands();
        typedef Idents::const_iterator IdIt;
        const Idents& des_types = filter.desTypes();
        for (IdIt i = des_types.begin(); i != des_types.end(); ++i) {
            Commands& cmds = cmds_map[*i];
            typedef Commands::const_iterator CIt;
            for (CIt j = rule_cmds.begin(); j != rule_cmds.end(); ++j) {
                cmds.push_back(*j);
            }
        }
    }
}

