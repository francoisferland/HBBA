///
/// \file say_events_observer.cpp A Say-related events generator.

#include <hbba_msgs/DesiresSet.h>
#include <hbba_msgs/Event.h>
#include <rt_audio_ros/AudioStream.h>
#include <ros/ros.h>
#include <vector>

namespace iw
{
    /// \brief Say-class events observer and generator.
    ///
    /// Subscribes to the current set of desires, filters it for the first
    /// Say-class events, and publishes "ACC_ON" events when the robot finished
    /// to speak.
    /// Will not repeat "ACC_ON" publications, only when transitions are
    /// detected.
    /// Notice that we do not care if a desire isn't part of the the Intention
    /// set.
    /// A desire can be accomplished even if it's not part of the robot's
    /// current intention.
    ///
    /// Parameters:
    ///  - say_class:       Class name of Say desire class.
    ///                     Default: "Say".
    ///  - pause_duration:  Pause in audio stream indicating the speech 
    ///                     synthesizer end. 
    ///                     Default: 0.5 s.
    ///
    /// Topics:
    ///  - desires_set:     The current active desires set as given by IW.
    ///  - events:          HBBA events output.
    ///
    class SayEventsObserver
    {
    private:
        ros::NodeHandle                 n_;
        ros::Subscriber                 sub_desires_;
        ros::Subscriber                 sub_audio_;
        ros::Publisher                  pub_events_;
        ros::Timer                      timer_;

        std::string                     say_class_;
        ros::Duration                   pause_duration_;
        ros::Time                       last_audio_;

        std::vector<hbba_msgs::Desire>  desires_;
        hbba_msgs::Desire               d_;

        bool                            synthesize_;

    public:
        /// \brief Constructor.
        ///
        /// \param n  Node handle for topics.
        /// \param np Node handle for parameters.
        ///
        SayEventsObserver(ros::NodeHandle& n, ros::NodeHandle& np)
        {
            np.param("say_class", say_class_, std::string("Say"));

            double pd;
            np.param("pause_duration", pd, 0.5);
            pause_duration_ = ros::Duration(pd);

            n_ = n;

            sub_desires_ = n.subscribe(
                "desires_set", 
                10, 
                &SayEventsObserver::desiresCB,
                this);

            sub_audio_ = n.subscribe(
                "audio", 
                10, 
                &SayEventsObserver::audioCB,
                this);

            pub_events_ = n.advertise<hbba_msgs::Event>("events", 10);
        }

    private:
        void desiresCB(const hbba_msgs::DesiresSet::ConstPtr& msg)
        {
            desires_ = msg->desires;
            detectEvents();
        }

        void detectEvents()
        {
            if(!synthesize_)
            {
                // Look for active Say desire in the current desires set.
                // and produce an ACC_ON event.
                typedef std::vector<hbba_msgs::Desire>::const_iterator DesIt;
                for (DesIt i = desires_.begin(); i != desires_.end(); ++i) {
                    const hbba_msgs::Desire& d = *i;
                    if (d.type != say_class_) {
                        continue;
                    }

                    d_ = d;

                    timer_ = n_.createTimer(
                        ros::Duration(0.1), 
                        &SayEventsObserver::timerCB, 
                        this,
                        true);

                    synthesize_ = true;
                    break;
                }
            }
        }

        void audioCB(const rt_audio_ros::AudioStream::ConstPtr& msg)
        {
            last_audio_ = ros::Time::now();
        }

        void timerCB(const ros::TimerEvent&)
        {
            if ((ros::Time::now() - last_audio_) >= pause_duration_) {
                hbba_msgs::Event evt;
                evt.desire      = d_.id; 
                evt.desire_type = say_class_;
                evt.type        = hbba_msgs::Event::ACC_ON;
                pub_events_.publish(evt);
                synthesize_ = false;
            } else {
                timer_ = n_.createTimer(
                    ros::Duration(0.1), 
                    &SayEventsObserver::timerCB, 
                    this,
                    true);
            }
        }

    };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "say_events_observer");

    ros::NodeHandle n, np("~");
    iw::SayEventsObserver node(n, np);

    ros::spin();

    return 0;
}

