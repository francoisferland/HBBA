#ifndef EVENTS_FILTER_HPP
#define EVENTS_FILTER_HPP

#include <hbba_msgs/Event.h>
#include <ros/ros.h>
#include <boost/function.hpp>
#include <string>

namespace iw
{
    /// \brief An IW events filter for a single desire ID with callback.
    ///
    /// An instance of this class automatically subscribes to the "event" topic
    /// in the provided handle's namespace.
    /// This topic will probably need to be remapped to the main events output,
    /// usually "/hbba/events".
    ///
    /// Currently only works for either all events relay or exploitation
    /// timeouts, full interface will be defined later.
    ///
    /// TODO: Remove event-specific callbacks ?
    ///
    class EventsFilter
    {
	public:
        /// \brief Constructor.
        ///
        /// \param n Node handle to use for topic registration.
        /// \param id Desire id to filter with.
        EventsFilter(ros::NodeHandle& n, const std::string& id);

        /// \brief Register a callback for all events related to a specific 
        /// desire.
        ///
        /// \brief fun A class member function pointer.
        /// \brief obj The class instance to use for this callback.
        template <class T>
        void allEventsCB(void (T::*fun)(const hbba_msgs::Event&), T* obj)
        {
            all_cb_ = boost::bind(fun, obj, _1);
        }

        /// \brief Register a callback for timeout on exploitation.
        ///
        /// See also resetExpTimeout() for usage.
        template <class T>
        void expTimeoutCB(void (T::*fun)(const hbba_msgs::Event&), T* obj)
        {
            exp_timeout_cb_ = boost::bind(fun, obj, _1);
        }

        /// \brief Reset the timer for exploitation timeout events.
        ///
        /// At instance initialization, the exploitation timer is not set.
        /// If a timeout is set with this method, an event will be generated if
        /// this fails to catch an EXP_ON event before the timeout has expired.
        /// The event generated will appear on both the all events callback and
        /// timeout-specific one.
        /// callback.
        ///
        /// TODO: Add auto-reset mode on EXP_ON events ?
        ///
        /// \param timeout: The time at which an event should be fired.
        void resetExpTimeout(const ros::Time& timeout);

        /// \brief Change the desire id to filter on.
        void id(const std::string& id) { id_ = id; }
        /// \brief Return the current desire id being filtered.
        const std::string& id() const { return id_; }

    private:
        void eventsCB(const hbba_msgs::Event::ConstPtr& msg);
        void expTimeoutTimerCB(const ros::TimerEvent&);

        ros::Subscriber sub_events_;
        ros::Timer exp_timeout_timer_;

        std::string id_;

        boost::function<void (const hbba_msgs::Event&)> all_cb_;
        boost::function<void (const hbba_msgs::Event&)> exp_timeout_cb_;

    };
}

#endif

