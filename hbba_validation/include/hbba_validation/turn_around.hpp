#ifndef TURN_AROUND_HPP
#define TURN_AROUND_HPP

#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

namespace hbba_validation
{
    /// \brief A simple behavior that rotates the robot on itself for 180
    /// degrees.
    ///
    /// Topics:
    ///
    ///  - odom:         Odometry input to figure out if a full turn has been
    ///                  performed.
    ///  - turn_trigger: An empty input that triggers a turnaround.
    ///  - turn_done:    An empty output that signals that the turn has
    ///                  finished.
    ///  - cmd_vel:      Twist output.
    ///
    /// Parameters:
    ///
    ///  - period: Command output period, in seconds.
    ///            Default: 0.10 s.
    ///  - td:     Absolute angular (theta) velocity.
    ///            Default: 0.25 rad/s.
    ///  - eps:    Error margin in final position check.
    ///            Default: 0.10 rad.
    ///
    class TurnAround
    {
    private:
        ros::Subscriber sub_odom_;
        ros::Subscriber sub_trigger_;
        ros::Publisher  pub_done_;
        ros::Publisher  pub_cmd_vel_;
        ros::Timer      timer_;

        double          td_;
        double          eps_;

        double          cur_yaw_;
        double          target_yaw_;

        bool            active_;

    public:
        /// \brief Constructor.
        ///
        /// \param n  Node handle for topics.
        /// \param np Node handle for parameters.
        TurnAround(ros::NodeHandle& n, ros::NodeHandle& np);

    private:
        void odomCB(const nav_msgs::Odometry& msg);
        void triggerCB(const std_msgs::Empty&);
        void timerCB(const ros::TimerEvent&);

    };

}

#endif

