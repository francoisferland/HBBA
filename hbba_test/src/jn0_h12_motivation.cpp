#include <ros/ros.h>
#include <hbba_msgs/AddDesires.h>
#include <std_msgs/Time.h>
#include <geometry_msgs/PoseStamped.h>

namespace hbba_test
{
    /// \brief Combined motivations for Johnny 0 tests.
    class Motivations
    {
    public:
        Motivations(): desires_(req_.request.desires), locate_int_(0)
        { 
            ros::service::waitForService("add_desires");
            ROS_INFO("Motivation starting ...");
            srv_add_des_ = 
                n_.serviceClient<hbba_msgs::AddDesires>("add_desires");

            // Same callback for both interruptions:
            sub_sound_ = n_.subscribe("sound", 1, 
                &Motivations::interruptCB, this);
            sub_poke_ = n_.subscribe("poke", 1,
                &Motivations::interruptCB, this);
            sub_sound_pose_ = n_.subscribe("sound_pose", 1,
                &Motivations::poseCB, this);

            ros::NodeHandle np("~");
            // Intensity lost at each update:
            np.param("int_grad", int_grad_, 0.10);
            // Intensity update period:
            double p;
            np.param("period", p, 0.5);
            
            timer_ = np.createTimer(ros::Duration(p),
                &Motivations::timerCB, this);

            desires_.resize(2);
            desires_[0].id = "jn0_locate_voice";
            desires_[0].type = "LocateVoice";
            desires_[0].utility = 1.0;
            desires_[0].intensity = 0.0;
            desires_[1].id = "jn0_track_ball";
            desires_[1].type = "BallTracking";
            desires_[1].utility = 1.0;
            desires_[1].intensity = 5.0; // Constant.

            updateDesires();
            ROS_INFO("Motivation ready.");
        }

    private:

        void interruptCB(const std_msgs::Time::ConstPtr& msg)
        {
            ros::Time now = ros::Time::now();
            // Ignore if the interruption is to old:
            if ((now - msg->data) > ros::Duration(0.5))
            {
                ROS_WARN("Ignoring old interruption.");
                return;
            }

            ROS_INFO("Interrupted!");

            // Up the voice locator intensity:
            locate_int_ = 10.0;
            updateDesires();
        }

        void poseCB(const geometry_msgs::PoseStamped&)
        {
            // Keep the locator intensity level high if we're successfully
            // tracking sound poses.
            locate_int_ = 10.0;
        }

        void timerCB(const ros::TimerEvent&)
        {
            const double EPS = 0.1;
            if (locate_int_ > EPS)
            {
                locate_int_ -= int_grad_;
                updateDesires();
            }
        }

        void updateDesires()
        {
            desires_[0].intensity = locate_int_;
            srv_add_des_.call(req_);
        }

        ros::NodeHandle n_;
        ros::ServiceClient srv_add_des_;
        ros::Subscriber sub_sound_;
        ros::Subscriber sub_poke_;
        ros::Subscriber sub_sound_pose_;
        ros::Timer timer_;

        hbba_msgs::AddDesires req_;
        std::vector<hbba_msgs::Desire>& desires_; // Will point to req_.
        double locate_int_;
        double int_grad_;

    };

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "jn0_h12_motivation");
    hbba_test::Motivations node;

    ros::spin();
    return 0;
}

