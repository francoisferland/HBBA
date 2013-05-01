#ifndef EVENTS_GENERATOR_HPP
#define EVENTS_GENERATOR_HPP

#include <hbba_msgs/DesiresSet.h>
#include <hbba_msgs/Intention.h>
#include <hbba_msgs/Event.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <map>

namespace iw
{
    /// \brief A module that generates messages from events happening in the
    /// Intention Workspace.
    ///
    /// Events are associated with desires.
    /// Desires are monitored for:
    ///
    ///  - When they appear or disappear from the active desires set;
    ///  - When they appear or disappear from the selected intention;
    ///  - When their exploitation starts or stops.
    ///
    /// Input topics:
    ///
    ///  - desires_set: The current active desires in the IW.
    ///  - intention: The active strategies selected by the solver.
    ///  - exploitation_match: Exploited desires as detected by
    ///    exploitation_matcher(s).
    ///
    /// Output topics:
    ///  - events: Messages representing a single event (hbba_msgs/Event).
    ///
    class EventsGenerator
    {
    public:
        /// \brief Constructor.
        ///
        /// \param n Node handle to use for topics and services.
        /// \param np Node handle to use for parameters.
        EventsGenerator(ros::NodeHandle& n, ros::NodeHandle& np);

    private:
        void desiresCB(const hbba_msgs::DesiresSet::ConstPtr& msg);
        void intentionCB(const hbba_msgs::Intention::ConstPtr& msg);
        void exploitationCB(const std_msgs::String::ConstPtr& msg);

        void event(const std::string& id, const unsigned char type);

        ros::Subscriber sub_desires_;
        ros::Subscriber sub_intention_;
        ros::Subscriber sub_exploitation_;
        ros::Publisher pub_events_;

        enum Flags
        {
            FLAG_NONE   = 0,
            FLAG_INT    = 1,
            FLAG_EXP    = 2
        };
        typedef std::map<std::string, int> DesMap;
        DesMap map_;

    };

}

#endif

