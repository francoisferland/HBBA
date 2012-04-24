#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <std_msgs/Float64.h>
#include <queue>

namespace topic_tools
{
    /// \brief Delay a message relay for N seconds.
    ///
    /// Subscribes to "~in", publishes to "~out".
    /// The delay time is changed with "~delay". 
    /// Doesn't modify the header.
    /// The delay is done inside the callback, so you need to be careful with
    /// too long delays.
    class TopicDelay
    {
    public:
        TopicDelay(): n_("~"), advertised_(false), delay_(0.0)
        {
            sub_in_ = n_.subscribe("in", 1, &TopicDelay::msgCB, this);
            sub_delay_ = n_.subscribe("delay", 1, &TopicDelay::delayCB, this);

            double period;
            n_.param("period", period, 0.05);

            timer_ = n_.createTimer(ros::Duration(period), 
                &TopicDelay::timerCB, this);
        }

        void msgCB(const topic_tools::ShapeShifter::ConstPtr& msg)
        {
            if (!advertised_)
            {
                pub_out_ = msg->advertise(n_, "out", 1, true);
                advertised_ = true;
            }

            queue_.push(std::make_pair(ros::Time::now(), msg));
        }

        void delayCB(const std_msgs::Float64::ConstPtr& msg)
        {
            delay_ = ros::Duration(msg->data);
        }

        void timerCB(const ros::TimerEvent&)
        {
            ros::Time now = ros::Time::now();
            while ((!queue_.empty()) && ((queue_.front().first + delay_) < now))
            {
               pub_out_.publish(queue_.front().second); 
               queue_.pop();
            }
        }

    private:
        ros::NodeHandle n_;
        ros::Subscriber sub_in_;
        ros::Subscriber sub_delay_;
        ros::Publisher pub_out_;
        ros::Timer timer_;

        bool advertised_;
        ros::Duration delay_;
    
        typedef std::pair<ros::Time, topic_tools::ShapeShifter::ConstPtr> 
            PairType;
        std::queue<PairType> queue_;
    };

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "topic_delay");

    topic_tools::TopicDelay node;
    ros::spin();

    return 0;
}

