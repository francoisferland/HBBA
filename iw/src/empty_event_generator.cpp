#include <hbba_msgs/DesiresSet.h>
#include <hbba_msgs/Event.h>
#include <std_msgs/Empty.h>
#include <ros/ros.h>

namespace iw
{
    /// \brief Generate a specific event based on a std_msgs/Empty trigger.
    ///
    /// Monitor a trigger topic (event_trigger) for empty messages.
    /// If desires of the specified class are in the current desires set, this
    /// node will produce events of the specified type (ACC_ON by default) for 
    /// these desires.
    ///
    /// Topics:  
    ///  - trigger:     Trigger input (std_msgs/Empty).
    ///  - desires_set: Current desire set input.
    ///  - events:      Events output.
    ///
    /// Parameters:
    ///  - desire_type: Desire class to filter on.
    ///                 Default: "", so invalid.
    ///  - event_type:  Event type int (see hbba_msgs/Event for values).
    ///                 Default: 8 (ACC_ON).
    ///
    class EmptyEventGenerator
    {
    private:
        ros::Subscriber sub_trigger_;
        ros::Subscriber sub_desires_;
        ros::Publisher  pub_events_;
        
        hbba_msgs::Event         base_event_;
        std::vector<std::string> cur_desires_;
            
    public:
        EmptyEventGenerator(ros::NodeHandle& n, ros::NodeHandle& np)
        {
            np.param("desire_type", 
                     base_event_.desire_type,
                     std::string(""));
            int e_type;
            np.param("event_type",
                     e_type,
                     int(hbba_msgs::Event::ACC_ON));
            base_event_.type = e_type;

            sub_trigger_ = n.subscribe("trigger", 
                                       1,
                                       &EmptyEventGenerator::triggerCB, 
                                       this);
            sub_desires_ = n.subscribe("desires_set",
                                       1,
                                       &EmptyEventGenerator::desiresCB,
                                       this);

            pub_events_ = n.advertise<hbba_msgs::Event>("events", 100);

        }

    private:
        void triggerCB(const std_msgs::Empty&)
        {
            typedef std::vector<std::string>::const_iterator It;
            for (It i = cur_desires_.begin(); i != cur_desires_.end(); ++i) {
                base_event_.desire = *i;
                pub_events_.publish(base_event_);
            }
        }

        void desiresCB(const hbba_msgs::DesiresSet& msg)
        {
            cur_desires_.clear();
            typedef std::vector<hbba_msgs::Desire>::const_iterator It;
            for (It i = msg.desires.begin(); i != msg.desires.end(); ++i) {
                if (i->type == base_event_.desire_type) {
                    cur_desires_.push_back(i->id);
                }
            }
        }

    };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "empty_event_generator");

    ros::NodeHandle n, np("~");
    iw::EmptyEventGenerator node(n, np);

    ros::spin();

    return 0;
}
