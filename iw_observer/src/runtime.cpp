#include <iw_observer/runtime.hpp>

using namespace iw_observer;

Runtime::EventTypeMap Runtime::event_type_map_;

Runtime::Runtime(const Rules& rules): rules_(rules)
{
    if (event_type_map_.empty()) {
        event_type_map_["exp_on"]  = hbba_msgs::Event::EXP_ON;
        event_type_map_["exp_off"] = hbba_msgs::Event::EXP_OFF;
    }

    ros::NodeHandle n;

    sub_events_ = n.subscribe("events", 100, &Runtime::eventsCB, this);

    scl_add_ = n.serviceClient<hbba_msgs::AddDesires>(   "add_desires",
                                                         true);
    scl_del_ = n.serviceClient<hbba_msgs::RemoveDesires>("remove_desires", 
                                                         true);

    parseRules(rules);
}

void Runtime::addDesire(const hbba_msgs::Desire& d)
{
    hbba_msgs::AddDesires c;
    c.request.desires.push_back(d);

    scl_add_.call(c);
}

void Runtime::removeDesire(const std::string& id)
{
    hbba_msgs::RemoveDesires c;
    c.request.ids.push_back(id);

    scl_del_.call(c);
}

void Runtime::eventsCB(const hbba_msgs::Event::ConstPtr& msg)
{
}

void Runtime::parseRules(const Rules& rules)
{
    typedef Rules::const_iterator It;
    for (It i = rules.begin(); i != rules.end(); ++i) {
        // Rule* rule = *i;
        // 1. Parse the event type
        // 2. Look for/Add the class(es) to the event type entry.
        // 3. Add the commands to the class(es) entry.
    }
}

