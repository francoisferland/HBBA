///
/// \file rosgraph_monitor.cpp \brief Copied from rosgraph_gl and adapted to iw.
///

#include <iw/rosgraph_monitor.hpp>

using namespace iw;

impl::TopicHandler::TopicHandler(
    ros::NodeHandle& n, 
    const std::string& topic_name):
        topic_name_(topic_name),
        counter_(0)
{
    sub_ = n.subscribe(topic_name, 10, &TopicHandler::msgCB, this);
}

void impl::TopicHandler::msgCB(const topic_tools::ShapeShifter::ConstPtr&)
{
    ++counter_;
}

const std::string& impl::TopicHandler::topicName() const
{
    return topic_name_;
}

int impl::TopicHandler::counter() const
{
    return counter_;
}

void impl::TopicHandler::resetCounter()
{
    counter_ = 0;
}

RosgraphMonitor::RosgraphMonitor(ros::NodeHandle& n, ros::NodeHandle& np)
{
    if (np.hasParam("topics")) {
        XmlRpc::XmlRpcValue array;
        np.getParam("topics", array);
        if (array.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_ERROR("'topics' parameter is not an array!");
            return;
        }
        for (int i = 0; i < array.size(); ++i) {
            XmlRpc::XmlRpcValue topic_name = array[i];
            if (topic_name.getType() != XmlRpc::XmlRpcValue::TypeString) {
                ROS_ERROR("'topics' element is not a string, skipping.");
                continue;
            }
            topic_handlers_.push_back(TopicHandlerPtr(new TopicHandler(
                n,
                topic_name)));
        }
    } else {
        ROS_WARN("No topic names provided ('topics' parameter not found).");
        return;
    }

    pub_events_ = n.advertise<RosgraphEvents>("rosgraph_events", 10);

    double p;
    np.param("period", p, 0.10);
    timer_ = n.createTimer(ros::Duration(p), &RosgraphMonitor::timerCB, this);
}

void RosgraphMonitor::timerCB(const ros::TimerEvent&)
{
    RosgraphEvents events;
    typedef TopicHandlers::iterator It;
    for (It i = topic_handlers_.begin(); i != topic_handlers_.end(); ++i) {
        TopicHandlerPtr th = *i;
        if (th->counter()) {
            RosgraphEvent e;
            e.topic_name = th->topicName();
            e.activity   = th->counter();
            events.events.push_back(e);
            th->resetCounter();
        }
    }

    if (events.events.size()) {
        pub_events_.publish(events);
    }
}

