#include <hbba_validation/turn_around.hpp>
#include <tf/tf.h>
#include <angles/angles.h>

using namespace hbba_validation;

TurnAround::TurnAround(ros::NodeHandle& n, ros::NodeHandle& np):
    active_(false)
{
    np.param("td",  td_,  0.25);
    np.param("eps", eps_, 0.10);

    sub_odom_    = n.subscribe("odom",         1, &TurnAround::odomCB,    this);
    sub_trigger_ = n.subscribe("turn_trigger", 1, &TurnAround::triggerCB, this);

    pub_done_    = n.advertise<std_msgs::Empty>("turn_done", 1);
    pub_cmd_vel_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    double p;
    np.param("period", p, 0.10);
    timer_ = np.createTimer(ros::Duration(p), &TurnAround::timerCB, this);
}

void TurnAround::odomCB(const nav_msgs::Odometry& msg)
{
    cur_yaw_ = angles::normalize_angle(tf::getYaw(msg.pose.pose.orientation));
}

void TurnAround::triggerCB(const std_msgs::Empty&)
{
    target_yaw_ = angles::normalize_angle(cur_yaw_ + M_PI);
    active_ = true;
}

void TurnAround::timerCB(const ros::TimerEvent&)
{
    if (!active_) {
        return;
    }

    if (fabs(cur_yaw_ - target_yaw_) < eps_) {
        std_msgs::Empty done;
        pub_done_.publish(done);
        active_ = false;
        return;
    }

    geometry_msgs::Twist cmd;
    cmd.angular.z = cur_yaw_ > target_yaw_ ? -td_ : td_;
                                        
    pub_cmd_vel_.publish(cmd);
}

