#ifndef UNLOCK_DOOR_HPP
#define UNLOCK_DOOR_HPP

#include "state_machine.hpp"
#include "cardreader_localizer_openni.hpp"
#include "cardreader_localizer_imv.hpp"
#include <jn0_arm_tools/arm_control_imp.hpp>
#include <jn0_arm_tools/point_at_trajectory.hpp>
#include <tf/transform_listener.h>
#include <ros/ros.h>

namespace hbba_validation
{
    /// \brief A behavior node that unlock doors with cardreaders at the 3IT.
    ///
    /// Assuming that a card is held in IRL-1's left gripper and a cardreader
    /// is visible in the Kinect's field of view, this node will point the card
    /// in the direction of the cardreader.
    /// The head is instructed to look at the cardreader.
    /// The mobile base is also commanded to get nearer to the wall.
    /// Return both the arm and head in a neutral position if the red LED of the
    /// cardreader is not visible.
    ///
    /// Operates a simple state machine:
    ///
    ///  - WAIT: Initial state, waits for a valid cardreader detection to
    ///          go to SEEK.
    ///  - SEEK: Control the robot toward the cardreader until a timeout event
    ///          occurs (time since last valid cardreader detection) to go back
    ///          to WAIT, or a valid green LED detection occured (indicates the
    ///          door has been unlocked).
    ///          
    /// 
    /// Topics (see jn0_arm_tools for arm-related topics):
    ///  - image:           RGB image produced by the Kinect camera.
    ///  - depth:           Depth image produced by the Kinect camera.
    ///  - image_imv_raw:   Raw IMV image used by CardreaderLocalizerIMV.
    ///  - cmd_vel:         Twist commands output for the mobile base.
    ///  - look_at_pose:    Look at commands, is a straight publication of the
    ///                     latest cardreader pose.
    ///  - ~cardreader_pose Test input for cardreader poses, produces a valid
    ///                     detection event.
    ///  - unlock_done:     A std_msgs/Empty output to signal that the door has
    ///                     been unlocked (green LED detected).
    ///
    /// Parameters: 
    ///  - period:      Control loop update period.
    ///                 Default: 0.10 s.
    ///  - timeout:     Period used to detect timeouts.
    ///                 Default: 2.00 s.
    ///  - fixed_frame: Reference frame in which all detected cardreader poses
    ///                 are transformed.
    ///                 Default: "/odom".
    ///  - robot_frame: The robot's reference frame.
    ///                 Default: "/base_link".
    ///  - offset_y:    Lateral offset that the base should keep between the
    ///                 cardreader and the robot's centerline (X axis).
    ///                 Measured along the Y axis, should ideally correspond to 
    ///                 the distance between the robot's centerline and the left 
    ///                 shoulder for a more natural arm lift.
    ///                 Default: 0.25 m.
    ///  - look_x:      Neutral look position in the robot's base link (in X).
    ///                 Default: 2.00 m.
    ///  - look_y:      Neutral look position in the robot's base link (in Y).
    ///                 Default: 0.00 m.
    ///  - look_z:      Neutral look position in the robot's base link (in Z).
    ///                 Default: 1.00 m, slightly tilted down.
    ///  - lin_k:       Linear velocity mobile base location error coefficient.
    ///                 Default: 0.2.
    ///  - ang_k:       Angular velocity mobile base location error coefficient.
    ///                 Default: 0.2.
    ///  - lin_max:     Maximum linear velocity (minimum is always zero).
    ///                 Default: 0.1 m/s.
    ///  - ang_max:     Maximum angular velocity (will be bounded between
    ///                 (-max, +max).
    ///                 Default: 0.3 rad/s.
    ///  - imp_k:       Cartesian trajectory impedance coefficient K.
    ///                 Default: 35.0.
    ///  - arm_dist:    Arm extension distance.
    ///                 Default: 0.50 m.
    ///  - arm_vel:     Arm extension velocity.
    ///                 Default: 0.25 m/s^2.
    ///              
    /// CardreaderLocalizerOpenNI-related parameters are in "~localizer_openni",
    /// and IMV-related ones in "~localizer_imv".
    class UnlockDoor
    {
    private:
        ros::Publisher        pub_cmd_vel_;
        ros::Publisher        pub_look_at_;
        ros::Publisher        pub_done_;
        ros::Subscriber       sub_pose_;
        ros::Timer            timer_;
        tf::TransformListener tf_;

        CardreaderLocalizerOpenNI localizer_openni_;
        CardreaderLocalizerIMV    localizer_imv_;
        
        boost::scoped_ptr<jn0_arm_tools::ArmStateSubscriber> arm_state_;
        boost::scoped_ptr<jn0_arm_tools::ArmControlImp>      arm_ctrl_;
        boost::scoped_ptr<jn0_arm_tools::PointAtTrajectory>  arm_point_at_;
        jn0_arm_tools::LinearCartesianTrajectory             arm_traj_;

        enum State {
            STATE_WAIT = 0,
            STATE_SEEK,
            STATE_DONE,
            STATE_SIZE
        };

        enum Events {
            EVENT_VALID = 0,
            EVENT_INVALID,
            EVENT_TIMEOUT,
            EVENT_GREEN,
            EVENT_SIZE
        };

        typedef StateMachine<UnlockDoor> SM;
        SM sm_;

        ros::Duration timeout_;
        std::string   fixed_frame_;
        std::string   robot_frame_;
        double        offset_y_;
        double        look_x_;
        double        look_y_;
        double        look_z_;
        double        lin_k_;
        double        ang_k_;
        double        lin_max_;
        double        ang_max_;
        double        imp_k_;
        double        arm_dist_;
        double        arm_vel_;

        ros::Time                  timeout_start_;
        geometry_msgs::PoseStamped cur_pose_;

    public:
        /// \brief Constructor.
        ///
        /// \param n  Node handle for topics.
        /// \param np Node handle for parameters.
        UnlockDoor(ros::NodeHandle& n, ros::NodeHandle& np);

    private:
        void validCB(const geometry_msgs::PoseStamped& pose);
        void invalidCB();
        void greenCB(const sensor_msgs::Image&, double, double);

        SM::Handle stateWait();
        SM::Handle stateSeek();
        SM::Handle stateDone();
       
        void timerCB(const ros::TimerEvent&);

        void produceVel();

    };
}

#endif

