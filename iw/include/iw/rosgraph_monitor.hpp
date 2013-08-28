///
/// \file rosgraph_client.hpp \brief Copied from rosgraph_gl and adapted to iw.
/// 

#ifndef ROSGRAPH_MONITOR_HPP
#define ROSGRAPH_MONITOR_HPP

#include <hbba_msgs/RosgraphEvents.h>
#include <topic_tools/shape_shifter.h>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

namespace iw
{
    namespace impl
    {
        /// \brief Topic subscription handler for RosgraphMonitor.
        class TopicHandler
        {
        private:
            std::string     topic_name_;
            ros::Subscriber sub_;
            int             counter_;

        public:
            /// \brief Constructor.
            ///
            /// Automatically subscribes to the given topic name through the
            /// provided node handle. 
            /// The internal counter is set to 0 and incremented each time a
            /// message is received.
            ///
            /// \param n          Node handle to use for subscription.
            /// \param topic_name The subscribed topic's name.
            ///
            TopicHandler(ros::NodeHandle& n, const std::string& topic_name);

            /// \brief Return this handler's topic name.
            ///
            /// This is the value as configured in the constructor, not as 
            /// resolved by the node handle.
            const std::string& topicName() const;

            /// \brief Return the current counter value.
            int counter() const;

            /// \brief Reset the counter value to 0.
            void resetCounter();

        private:
            void msgCB(const topic_tools::ShapeShifter::ConstPtr&);
        };
    } 

    /// \brief A topic-based rosgraph monitor.
    ///
    /// This node monitors messages on a (pre-configured) set of topics and
    /// an activity log.
    /// On a regular basis, the activity log for the monitoring period is sent
    /// on an output topic and flushed for the next period.
    ///
    /// Parameters:
    /// 
    ///  - topics: An array of strings, each one a topic to monitor.
    ///            Default: empty.
    ///  - period: The time period between log sendings.
    ///            Default: 0.1 s.
    /// 
    /// Topics:
    ///  - rosgraph_activity: A vector of RosgraphEvent structs.
    ///
    class RosgraphMonitor
    {
    private:
        typedef impl::TopicHandler              TopicHandler;
        typedef boost::shared_ptr<TopicHandler> TopicHandlerPtr;
        typedef std::vector<TopicHandlerPtr>    TopicHandlers;

        ros::Publisher pub_events_;
        ros::Timer     timer_;
        TopicHandlers  topic_handlers_;

        boost::function<void (hbba_msgs::RosgraphEvents&)> events_cb_;

    public:
        /// \brief Constructor.
        ///
        /// \param n  Node handle for topics.
        /// \param np Node handle for parameters.
        RosgraphMonitor(ros::NodeHandle& n, ros::NodeHandle& np);

        /// \brief Add a new topic handler to the monitored set.
        ///
        /// \param n          Reference node handle.
        /// \param topic_name Topic to monitor.
        void addTopic(ros::NodeHandle& n, const std::string& topic_name);

        /// \brief Register a class method as an internal events callback.
        template <class T>
        void registerCB(void (T::*fun)(const hbba_msgs::RosgraphEvents&), T* obj)
        {
            events_cb_ = boost::bind(fun, obj, _1);
        }

    private:
        void timerCB(const ros::TimerEvent&);

    };
}

#endif

