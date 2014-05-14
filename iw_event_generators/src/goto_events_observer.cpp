///
/// \file goto_events_observer.cpp A GoTo-related events generator.
/// Essentially a prototype for a very specific type of desire and its
/// accomplishement event.
/// Might pave the way for a more generic events observer later.

#include <hbba_msgs/DesiresSet.h>
#include <hbba_msgs/Event.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <sstream>
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
    ///
    /// Note: A goal parameters are cached, so events for desires that do not 
    /// change names might not be properly tracked.
    ///
    /// Monitors TF for the robot's position, see "robot_frame" for details.
    /// Currently, only the position is tracked, orientation has no effect.
    //
    /// Parameters:
    ///  - goto_class:  Class name of GoTo desire class.
    ///                 Default: "GoTo".
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
        typedef tf::Stamped<tf::Pose>              StampedPose;
        typedef std::map<std::string, StampedPose> Model; 

        ros::Subscriber                sub_desires_;
        ros::Publisher                 pub_events_;
        ros::Timer                     timer_;
        tf::TransformListener          tf_; 

        std::string                    goto_class_;
        std::string                    robot_frame_;
        double                         goal_eps_;

        std::vector<hbba_msgs::Desire> desires_;

        // Contains the last set of desires where the goal has being reached, 
        // used to detect ON->OFF/OFF->ON transitions:
        Model                          model_;

    public:
        /// \brief Constructor.
        ///
        /// \param n  Node handle for topics.
        /// \param np Node handle for parameters.
        ///
        GotoEventsObserver(ros::NodeHandle& n, ros::NodeHandle& np)
        {
            np.param("goto_class",  goto_class_,  std::string("GoTo"));
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
            desires_ = msg->desires;
            detectEvents();
        }

        void timerCB(const ros::TimerEvent&)
        {
            detectEvents();
        }

        void detectEvents()
        {
            // 1. Look for goals in the model that are no longer in reach,
            // produce ACC_OFF events for them, and remove them from the model.
            typedef Model::iterator ModelIt;
            ModelIt i = model_.begin();
            while (i != model_.end()) {
                const std::string& d_id = i->first;
                const StampedPose& pose = i->second;

                ModelIt last = i;
                ++i;

                if (!goalInReach(pose)) {
                    hbba_msgs::Event evt;
                    evt.desire      = d_id;
                    evt.desire_type = goto_class_;
                    evt.type        = hbba_msgs::Event::ACC_OFF;
                    pub_events_.publish(evt);

                    model_.erase(last);
                }
            }

            // 2. Look for active GoTo goals in the current desires set.
            //    Skip them if they are already in the model.
            //    If they're not and they are in reach, add them to the model 
            //    and produce an ACC_ON event.
            typedef std::vector<hbba_msgs::Desire>::const_iterator DesIt;
            for (DesIt i = desires_.begin(); i != desires_.end(); ++i) {
                const hbba_msgs::Desire& d = *i;
                if (d.type != goto_class_) {
                    continue;
                }

                if (model_.find(d.id) != model_.end()) {
                    continue;
                }

                // TODO: Add to a tracked new goals so parsing only has to be
                // done once.
                StampedPose pose;
                if (!parseGoal(d.params, pose)) {
                    ROS_ERROR(
                        "Cannot parse goal from desire '%s': '%s'",
                        d.id.c_str(),
                        d.params.c_str());
                    continue;
                }

                if (goalInReach(pose)) {
                    hbba_msgs::Event evt;
                    evt.desire      = d.id; 
                    evt.desire_type = goto_class_;
                    evt.type        = hbba_msgs::Event::ACC_ON;
                    pub_events_.publish(evt);

                    model_[d.id] = pose;
                }

            }
        }

        /// \brief Parse a JSON-encoded goal into a stamped pose.
        ///
        /// \return false if parsing failed, might leave invalid data in pose.
        bool parseGoal(const std::string& params, StampedPose& pose)
        {
            // NOTE: As JSON is a subset of YAML, it is perfectly valid to use a
            // YAML parser here.
           
            pose.stamp_ = ros::Time(0);

            try {
                std::istringstream doc(params);
                YAML::Parser parser(doc);
                YAML::Node node;
                while (parser.GetNextDocument(node)) {
                    double x, y, t;

                    node["frame_id"] >> pose.frame_id_;
                    node["x"]        >> x;
                    node["y"]        >> y;
                    node["t"]        >> t;

                    ROS_DEBUG(
                        "Parsed a new navigation goal: (%f, %f, %f)",
                        x,
                        y,
                        t);

                    pose.setOrigin(tf::Vector3(x, y, 0));
                    pose.setRotation(tf::createQuaternionFromYaw(t));
                }
            } catch (YAML::Exception&) {
                return false;
            }

            return true;

        }

        bool goalInReach(const StampedPose& pose)
        {
            // TODO: Consider caching the robot's pose?

            tf::StampedTransform robot;
            try {
                tf_.lookupTransform(
                    pose.frame_id_, 
                    robot_frame_, 
                    ros::Time(0),
                    robot); 
            } catch (tf::TransformException& e) {
                ROS_ERROR(
                    "Cannot get the robot's pose in the goal frame %s, "
                    "reason: %s.,",
                    pose.frame_id_.c_str(),
                    e.what());
                return false;
            }

            return (robot.getOrigin() - pose.getOrigin()).length() <= goal_eps_;
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

