#include <topic_filters_manager/gd_manager.hpp>
#include <std_msgs/Int64.h>

using namespace topic_filters_manager;

GenericDividerManager::GenericDividerManager()
{
}

void GenericDividerManager::parseFiltersParam()
{
    // TODO!
}

void GenericDividerManager::setRate(const std::string& name, int rate)
{
    HandlersMap::iterator i = map_.find(name);
    if (i == map_.end()) {
        ROS_DEBUG("Filter '%s' not found, creating on demand.", name.c_str());
        i = map_.insert(
                std::make_pair(name, 
                               FilterHandlerPtr(
                                   new FilterHandler(name)))).first;
    }

    FilterHandlerPtr& handler = i->second;
    handler->setRate(rate);
}

GenericDividerManager::FilterHandler::FilterHandler(const std::string& name):
    first_call_(true)
{
    ROS_DEBUG("Advertising 'divider_rate' in /%s...", name.c_str());
    ros::NodeHandle n("/" + name);
    pub_ = n.advertise<std_msgs::Int64>("divider_rate", 1, true);
}

void GenericDividerManager::FilterHandler::setRate(int rate)
{
    static std_msgs::Int64 msg;

    // Don't warn on first call, for some reason the number is always zero.
    if (!first_call_ && pub_.getNumSubscribers() == 0) {
        ROS_WARN("Divider rate topic '%s' has no subscribers.", 
                 pub_.getTopic().c_str());
    }

    msg.data = rate;
    pub_.publish(msg);

    first_call_ = false;
}

