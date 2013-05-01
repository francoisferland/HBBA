#include <iw/events_filter.hpp>

using namespace iw;

EventsFilter::EventsFilter(ros::NodeHandle& n, const std::string& id):
    id_(id)
{
    sub_events_ = n.subscribe("events", 100, &EventsFilter::eventsCB, this);
}

void EventsFilter::eventsCB(const hbba_msgs::Event::ConstPtr& msg)
{
    if (msg->desire != id_)
        return;

    if (all_cb_)
        all_cb_(*msg);

    if (msg->type == hbba_msgs::Event::EXP_ON)
        exp_timeout_timer_.stop();

}

void EventsFilter::resetExpTimeout(const ros::Time& timeout)
{
    ros::NodeHandle n;
    exp_timeout_timer_ = n.createTimer(
        timeout - ros::Time::now(),
        &EventsFilter::expTimeoutTimerCB,
        this,
        true); // One-shot.
}

void EventsFilter::expTimeoutTimerCB(const ros::TimerEvent&)
{
    hbba_msgs::Event evt;
    evt.desire = id_;
    evt.type = hbba_msgs::Event::EXP_TIMEOUT;

    if (exp_timeout_cb_)
        exp_timeout_cb_(evt);
    if (all_cb_)
        all_cb_(evt);

}

