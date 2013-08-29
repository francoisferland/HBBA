///
/// \file goto_events_observer.cpp A GoTo-related events generator.
/// Essentially a prototype for a very specific type of desire and its
/// accomplishement event.
/// Might pave the way for a more generic events observer later.

#include <hbba_msgs/DesiresSet.h>
#include <hbba_msgs/Event.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <map>

namespace iw
{
    /// \brief GoTo-class events observer and generator.
    ///
    /// Subscribes to the current set of desires, filters it for GoTo-class
    /// events, and publishes "ACC_ON" events when the robot is within close
    /// distance of the location to reach.
    /// Publishes "ACC_OFF" when the robot leaves the goal position and the
    /// desire is still active.
    /// Will not repeat "ACC_ON" or "ACC_OFF" publications, only when
    /// transitions are detected.
    /// Notice that we do not care if a desire isn't part of the the Intention
    /// set.
    /// A desire can be accomplished even if it's not part of the robot's
    /// current intention.
    /// Monitors TF for the robot's position, see "robot_frame" for details.
    //
    /// Parameters:
    ///  - robot_frame: The robot's frame of reference.
    ///                 Used to measure distance between the goal and the
    ///                 robot's current position.
    ///                 This frame will be transformed in the goal's reference
    ///                 frame, usually a fixed one such as "/map".
    ///                 Default: "base_link"
    ///  - goal_eps:    Minimal distance from the goal to be considered as
    ///                 reached.
    ///                 Default: 2.0 m.
    ///  - period:      Update period for event detection.
    ///                 Note: events detection is performed on each reception of
    ///                 a new desires set.
    ///                 Default: 1.0 s.
    ///
    /// Topics:
    ///  - desires_set: The current active desires set as given by IW.
    ///  - events:      HBBA events output.
    ///
    class GotoEventsObserver
    {
    private:
        ros::Subscriber         sub_desires_;
        ros::Publisher          pub_events_;
        ros::Timer              timer_;
        tf::TransformListener   tf_; 

        std::string             robot_frame_;
        double                  goal_eps_;

        hbba_msgs::DesiresSet   desires_set_;

        // Contains the last set of desires where the goal is in reached, used
        // to detect ON->OFF/OFF->ON transitions:
        typedef tf::Stamped<tf::Pose> StampedPose;
        typedef std::map<std::string, StampedPose> Model; 
        Model model_;

    public:
        /// \brief Constructor.
        ///
        /// \param n  Node handle for topics.
        /// \param np Node handle for parameters.
        ///
        GotoEventsObserver(ros::NodeHandle& n, ros::NodeHandle& np)
        {
            np.param("robot_frame", robot_frame_, std::string("base_link"));
            np.param("goal_eps",    goal_eps_,    2.0);

            sub_desires_ = n.subscribe(
                "desires_set", 
                10, 
                &GotoEventsObserver::desiresCB,
                this);

            pub_events_ = n.advertise<hbba_msgs::Event>("events", 10);

            double p;
            np.param("period", p, 1.0);
            timer_ = n.createTimer(
                ros::Duration(p), 
                &GotoEventsObserver::timerCB, 
                this);

        }

    private:
        void desiresCB(const hbba_msgs::DesiresSet::ConstPtr& msg)
        {
            desires_set_ = *msg;
            detectEvents();
        }

        void timerCB(const ros::TimerEvent&)
        {
            detectEvents();
        }

        void detectEvents()
        {
            // TODO:
            // 1. Look for goals in the model that are no longer in reach,
            // produce ACC_OFF events for them, and remove them from the model.
            // 2. Look for active GoTo goals in the current desires set.
            //    If they're in reach, check if they're already in the model.
            //    If they're not, add them and produce an ACC_ON event.
        }

        /// \brief Parse a JSON-encoded goal into a stamped pose.
        void parseGoal(const std::string& params, StampedPose& pose)
        {
            // TODO: Find a good JSON parser.
            pose.stamp_ = ros::Time(0);
        }

    };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "goto_events_observer");

    ros::NodeHandle n, np("~");
    iw::GotoEventsObserver node(n, np);

    ros::spin();

    return 0;
}

