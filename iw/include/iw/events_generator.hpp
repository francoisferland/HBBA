#ifndef EVENTS_GENERATOR_HPP
#define EVENTS_GENERATOR_HPP

#include "exploitation_matcher.hpp"
#include "rosgraph_monitor.hpp"
#include <hbba_msgs/DesiresSet.h>
#include <hbba_msgs/Intention.h>
#include <hbba_msgs/Event.h>
#include <hbba_msgs/CreateExploitationMatcher.h>
#include <hbba_msgs/RegisterTopicExploitationMatches.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
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
    /// Parameters:
    ///  - exp_timeout: Time used for exploitation deactivation detection.
    ///    If a desire has not been exploited for this duration, an 
    ///    EXP_OFF event is generated.
    ///    EXP_OFF events are generated only if there has been a previous
    ///    EXP_ON event.
    ///    Default: 0.5 s.
    ///  - rosgraph_monitor/topics: A list of topics to monitor for activity and 
    ///    thus perception-related desire exploitation. 
    ///  - exploitation_matches: A dictionary of exploitation matches.
    ///    This parameter is expected in the same namespace as this node, so
    ///    '/hbba', usually.
    ///    The format is as such:
    ///
    ///       exploitation_matches:
    ///           - topic_name: 'full_topic_name_1'
    ///             matches: 
    ///              - priority: priority_value_1
    ///                desire_type: ['desire_type_1', 'desire_type_2', ...]
    ///              - priority: priority_value_2
    ///                desire_type: ['desire_type_3', ...]   
    ///           - topic_name: 'full_topic_name_2'
    ///             matches: ...    
    ///
    ///    See 'hbba_synth' for automatic generation of this parameter.
    ///
    /// Services:
    ///  - create_exploitation_matcher: Create an exploitation matcher for
    ///    arbitration output topics.
    ///  - create_topic_exploitation_matcher: Create an exploitation matcher for
    ///    standard (usually perception) topics based on activity monitoring.
    ///
    /// Input topics:
    ///  - desires_set: The current active desires in the IW.
    ///  - intention: The active strategies selected by the solver.
    ///  - exploitation_match: Exploited desires as detected by
    ///    exploitation_matcher(s).
    ///  - all topics defined in rosgraph_monitor/topics.
    ///
    /// Output topics:
    ///  - events: Messages representing a single event (hbba_msgs/Event).
    ///
    /// See also ExploitationMatcher for other input/output topics, and
    /// CreateExploitationMatcher's interface for sub-namespace details.
    class EventsGenerator
    {
    public:
        /// \brief Constructor.
        ///
        /// \param n Node handle to use for topics and services.
        /// \param np Node handle to use for parameters.
        EventsGenerator(ros::NodeHandle& n, ros::NodeHandle& np);

        ~EventsGenerator();

    private:
        void desiresCB(const hbba_msgs::DesiresSet::ConstPtr& msg);
        void intentionCB(const hbba_msgs::Intention::ConstPtr& msg);
        bool cemCB(
            hbba_msgs::CreateExploitationMatcher::Request& req,
            hbba_msgs::CreateExploitationMatcher::Response& res);
        bool ctemCB(
            hbba_msgs::RegisterTopicExploitationMatches::Request& req,
            hbba_msgs::RegisterTopicExploitationMatches::Response& res);
        void timerCB(const ros::TimerEvent&);

        void exploitationCB(const std::string& id);
        void rosgraphEventsCB(const hbba_msgs::RosgraphEvents& msg);

        void detectExpOff();
        void event(
            const std::string& id, 
            const std::string& d_type,
            const unsigned char type);

        /// \brief Create Exploitation match(es).
        void cem(
                const std::string& topic_name, 
                const std::vector<hbba_msgs::ExploitationMatch>& matches);

        void parseExploitationMatches(const ros::NodeHandle& n);

        ros::Subscriber sub_desires_;
        ros::Subscriber sub_intention_;
        ros::ServiceServer srv_cem_;
        ros::ServiceServer srv_ctem_;
        ros::Publisher pub_events_;
        ros::Timer timer_;

        ros::Duration exp_timeout_;

        enum Flags
        {
            FLAG_NONE   = 0,    
            FLAG_INT    = 1,    // Appears in Intention set.
            FLAG_EXP    = 2,    // Is being exploited.
            FLAG_ACC    = 4     // As been accomplished as a goal.
        };
        struct DesireData
        {
            int         flags;
            std::string type;
            ros::Time   last_exp_;
        };
        typedef std::map<std::string, DesireData> Model;
        Model model_;

        std::list<ExploitationMatcher*> matchers_;

        boost::scoped_ptr<RosgraphMonitor> rosgraph_monitor_;
        typedef std::map< std::string, std::vector<std::string> > TEMMap;
        TEMMap tem_map_;  // Topic Exploitation matchers map (topic -> classes).

    };

}

#endif

