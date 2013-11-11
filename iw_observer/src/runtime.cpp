#include <iw_observer/runtime.hpp>

using namespace iw_observer;

Runtime::Runtime()
{
    ros::NodeHandle n;

    scl_add_ = n.serviceClient<hbba_msgs::AddDesires>(   "add_desires",
                                                         true);
    scl_del_ = n.serviceClient<hbba_msgs::RemoveDesires>("remove_desires", 
                                                         true);
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

