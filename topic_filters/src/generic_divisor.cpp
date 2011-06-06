#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <topic_tools/shape_shifter.h>
#include <topic_filters/SetDivisorRate.h>
#include <ros/ros.h>

namespace topic_filters
{
    /// \brief A switch/divisor generic topic filter nodelet.
    ///
    /// The rate of the divisor sets how many messages should be skipped.
    /// The default is 1.
    /// A rate of 1 republishes all messages, a rate 2 is half, a rate of 4 is
    /// a quarter, etc.
    ///
    /// A rate of less than 1 stops completely.
    ///
    /// In latch mode, the filter sends the last received message as soon
    /// as it's reactivated.
    class GenericDivisor: public nodelet::Nodelet
    {
    public:
        /// \brief Default constructor. Default rate is 1.
        GenericDivisor(): rate_(0), count_(0), advertised_(false) {}

        virtual void onInit()
        {
            const std::vector<std::string>& args = getMyArgv();
            if (args.size() < 2)
            {
                NODELET_ERROR("Filter called with less than 2 args"
                    " (topic_in, topic_out).");
                return;
            }

            n_ = getNodeHandle();
            sub_ = n_.subscribe(args[0], 10, &GenericDivisor::msgCB, this);
            output_name_ = args[1];
            
            ros::NodeHandle np = getPrivateNodeHandle();
            srv_rate_ = np.advertiseService("set_divisor_rate",
                &GenericDivisor::rateCB, this);
            np.param("latch", latch_, true);

        }

    private:
        void msgCB(const topic_tools::ShapeShifter::ConstPtr& msg)
        {
            if (!advertised_)
            {
                // We can't automatically advertise shape shifter topics
                // before getting a message (and its MD5) first.
                // Note that this filter couldn't actually change its type
                // after getting a first message, and that the output topic
                // will be advertised even if the filter is deactivated.
                //ROS_INFO("Advertising %s ...", output_name_.c_str());
                pub_ = msg->advertise(n_, output_name_, 10, latch_);
                advertised_ = true;
                // TODO Don't understand why, but at first the publisher think
                // there is no subscribers, dropping the first message. Latching
                // seems not working too (will need a retry).
                // TODO: Figure out if this works in any threading situation.
                while (pub_.getNumSubscribers() == 0) {
                    ros::Duration(0.1).sleep();
                }
            }

            // A rate of 0 will never publish messages, and we need to
            // avoid divisions by 0.
            if (rate_ && !(count_ % rate_))
            {
                //ROS_INFO("Publishing on %s ...", output_name_.c_str());
                pub_.publish(msg);
            }

            last_msg_ = msg;
            count_++;
        }

        bool rateCB(topic_filters::SetDivisorRate::Request& req,
            topic_filters::SetDivisorRate::Response&)
        {
            rate_ = req.divisor;
            if (rate_ < 0)
                rate_ = 0;

            if (rate_ > 0)
            {
                //ROS_INFO("Turning on %s", output_name_.c_str()); 
                if (latch_ && advertised_)
                {
                    //ROS_INFO("Releasing latched message.");
                    pub_.publish(last_msg_);
                }
            }
            else
            {
                //ROS_INFO("Turning off %s", output_name_.c_str());
            }

            return true;
        }

        int rate_;
        int count_;
        bool advertised_;
        bool latch_;
        std::string output_name_;

        ros::NodeHandle n_;
        ros::Subscriber sub_;
        ros::Publisher pub_;
        ros::ServiceServer srv_rate_;

        topic_tools::ShapeShifter::ConstPtr last_msg_;

    };
}

PLUGINLIB_DECLARE_CLASS(topic_filters, GenericDivisor,
    topic_filters::GenericDivisor, nodelet::Nodelet)

