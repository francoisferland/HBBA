#include <topic_tools/shape_shifter.h>
#include <topic_filters/SetDividerRate.h>
#include <std_msgs/Int64.h>
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include <deque>

namespace topic_filters
{
    /// \brief A switch/divider generic topic filter nodelet.
    ///
    /// The rate of the divider sets how many messages should be skipped.
    /// The default is 1.
    /// A rate of 1 republishes all messages, a rate 2 is half, a rate of 4 is
    /// a quarter, etc.
    ///
    /// A rate of less than 1 stops completely.
    ///
    /// In latch mode, the filter sends the last received message as soon
    /// as it's reactivated.
    class GenericDividerNode
    {
    public:
        /// \brief Default constructor. Default rate is 1.
        GenericDividerNode(ros::NodeHandle& n, 
                           ros::NodeHandle& np, 
                           const std::vector<std::string>& args): 
            n_(n),
            rate_(0), 
            count_(0), 
            advertised_(false) 
        {
            if (args.size() < 2)
            {
                ROS_ERROR("Filter called with less than 2 args"
                    " (topic_in, topic_out).");
                return;
            }

            sub_ = n_.subscribe(args[0], 10, &GenericDividerNode::msgCB, this);
            output_name_ = args[1];
            
            srv_rate_ = np.advertiseService("set_divider_rate",
                &GenericDividerNode::setRateCB, this);
            int ls;
            np.param("latch_size", ls, 1);
            latch_size_ = abs(ls);

            np.param("divider_rate", rate_, 0);

            // Alternative divider rate input as a simple topic:
            sub_rate_ = np.subscribe("divider_rate", 
                                     1, 
                                     &GenericDividerNode::rateCB, 
                                     this);

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
                // ROS_INFO("Advertising %s ...", output_name_.c_str());
                pub_ = msg->advertise(n_, output_name_, 10, latch_size_ > 0);
                advertised_ = true;
                // TODO Don't understand why, but at first the publisher think
                // there is no subscribers, dropping the first message. Latching
                // seems not working too (will need a retry).
                // TODO: Figure out if this works in any threading situation.
                // TODO: Disabled! the script engine get stuck if this happens.
                //while (pub_.getNumSubscribers() == 0) {
                //    ros::Duration(0.1).sleep();
                //}
            }

            // A rate of 0 will never publish messages, and we need to
            // avoid divisions by 0.
            if (rate_ && !(count_ % rate_))
            {
                //ROS_INFO("Publishing on %s ...", output_name_.c_str());
                pub_.publish(msg);
            }

            //last_msg_ = msg;
            latch_buffer_.push_back(msg);
            if (latch_buffer_.size() > latch_size_)
                latch_buffer_.pop_front();

            count_++;
        }

        bool setRateCB(topic_filters::SetDividerRate::Request& req,
            topic_filters::SetDividerRate::Response&)
        {
            rate_ = req.divider;

            updateRate();
            return true;
        }

        void rateCB(const std_msgs::Int64& msg)
        {
            rate_ = msg.data;
            updateRate();
        }

        void updateRate()
        {
            if (rate_ < 0)
                rate_ = 0;

            ROS_DEBUG("Switching rate to %i", rate_);

            if (rate_ > 0)
            {
                if ((latch_size_ > 0) && advertised_)
                {
                    while (!latch_buffer_.empty())
                    {
                        MsgPtr m = latch_buffer_.front();
                        pub_.publish(m);
                        latch_buffer_.pop_front();
                    }
                }
            }
            else
            {
                //ROS_INFO("Turning off %s", output_name_.c_str());
            }
        }

        ros::NodeHandle n_;
        ros::Subscriber sub_;
        ros::Publisher pub_;
        ros::ServiceServer srv_rate_;
        ros::Subscriber sub_rate_;

        int rate_;
        int count_;
        bool advertised_;
        size_t latch_size_;
        std::string output_name_;

        typedef topic_tools::ShapeShifter::ConstPtr MsgPtr;
        typedef std::deque<MsgPtr> QueueType;
        QueueType latch_buffer_;

    };

}

