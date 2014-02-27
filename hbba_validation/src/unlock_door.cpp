#include <hbba_validation/unlock_door.hpp>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>

using namespace hbba_validation;

UnlockDoor::UnlockDoor(ros::NodeHandle& n, ros::NodeHandle& np):
    localizer_openni_(n, ros::NodeHandle(np, "localizer_openni")),
    localizer_imv_(n, 
                   ros::NodeHandle(np, "localizer_imv"), 
                   CardreaderLocalizerIMV::MODE_BOTH)
{
    np.param("fixed_frame", fixed_frame_, std::string("/odom"));
    np.param("robot_frame", robot_frame_, std::string("/base_link"));
    np.param("offset_y",    offset_y_,    0.25);
    np.param("look_x",      look_x_,      2.00);
    np.param("look_y",      look_y_,      0.00);
    np.param("look_z",      look_z_,      1.00);
    np.param("imv_dist",    imv_dist_,    0.75);
    np.param("lin_k",       lin_k_,       0.50);
    np.param("ang_k",       ang_k_,       0.50);
    np.param("lin_max",     lin_max_,     0.10);
    np.param("ang_max",     ang_max_,     0.30);
    np.param("imp_k",       imp_k_,       35.0);
    np.param("arm_dist",    arm_dist_,    0.50);
    np.param("arm_vel",     arm_vel_,     0.25);

    localizer_openni_.registerValidCB(&UnlockDoor::validCB, this);
    localizer_openni_.registerInvalidCB(&UnlockDoor::invalidCB, this);
    localizer_imv_.registerCB(&UnlockDoor::imvCB, this);


    arm_state_.reset(new jn0_arm_tools::ArmStateSubscriber("L_", n, np));
    arm_ctrl_.reset(new jn0_arm_tools::ArmControlImp("L_", 
                                                     arm_state_.get(), 
                                                     true));
    arm_point_at_.reset(new jn0_arm_tools::PointAtTrajectory(arm_state_.get(),
                                                             arm_dist_,
                                                             arm_vel_));

    SM::States states;
    states.push_back(&UnlockDoor::stateWait);
    states.push_back(&UnlockDoor::stateSeek);
    states.push_back(&UnlockDoor::stateDone);
    SM::Transitions transitions;
    SM::generateTransitionsMatrix(STATE_SIZE, EVENT_SIZE, transitions);
    transitions[STATE_WAIT][EVENT_VALID]   = STATE_SEEK;
    transitions[STATE_SEEK][EVENT_GREEN]   = STATE_DONE;
    transitions[STATE_SEEK][EVENT_TIMEOUT] = STATE_WAIT;
    sm_ = SM(states, transitions, this);

    double p;
    np.param("period", p, 0.10);
    timer_ = n.createTimer(ros::Duration(p), &UnlockDoor::timerCB, this);
    np.param("timeout", p, 2.00);
    timeout_ = ros::Duration(p);

    pub_cmd_vel_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    pub_look_at_ = n.advertise<geometry_msgs::PoseStamped>("look_at_pose", 1);
    pub_done_    = n.advertise<std_msgs::Empty>("unlock_done", 1);

    sub_pose_    = np.subscribe("cardreader_pose", 
                                1, 
                                &UnlockDoor::validCB, 
                                this);
}

void UnlockDoor::validCB(const geometry_msgs::PoseStamped& pose)
{
    ROS_DEBUG_THROTTLE(1.0, "Got a valid detection.");
    // Save the detected pose in the fixed frame.
    try {
        tf::Stamped<tf::Pose> pose_tf;
        tf::poseStampedMsgToTF(pose, pose_tf);
        tf::StampedTransform t;
        tf_.lookupTransform(fixed_frame_, pose.header.frame_id, ros::Time(), t);
        pose_tf.setData(t * pose_tf);
        pose_tf.frame_id_ = fixed_frame_;
        pose_tf.stamp_    = t.stamp_;
        tf::poseStampedTFToMsg(pose_tf, cur_pose_);
        timeout_start_ = ros::Time::now(); // Starts timeout counter.
        sm_.pushEvent(EVENT_VALID);
    } catch (tf::TransformException e) {
        ROS_ERROR("Could not transform detected cardreader pose in the "
                  "fixed frame, reason: %s",
                  e.what());
        sm_.pushEvent(EVENT_INVALID);
    }
}

void UnlockDoor::invalidCB()
{
    ROS_DEBUG_THROTTLE(1.0, "Got an invalid detection.");
    sm_.pushEvent(EVENT_INVALID);
}

void UnlockDoor::imvCB(const sensor_msgs::Image& img, 
                             double              pan, 
                             double              tilt, 
                             bool                g)
{
    if (g) {
        ROS_DEBUG_THROTTLE(1.0, "Got a green LED detection on IMV.");
        sm_.pushEvent(EVENT_GREEN);
    } else {
        ROS_DEBUG_THROTTLE(1.0, "Got a red LED detection on IMV.");
        // Only produce a valid detection if the current pose is old to avoid
        // replacing OpenNI-generated ones:
        if ((ros::Time::now() - cur_pose_.header.stamp) < timeout_) {
            return;
        }
        // NOTE: Baked-in transform, assuming the IMV frame's X points in front
        // of the robot.
        geometry_msgs::PoseStamped pose;
        pose.header             = img.header;
        pose.pose.position.x    = imv_dist_ * cos(-pan) * (1.0 - sin(tilt));
        pose.pose.position.y    = imv_dist_ * sin(-pan) * (1.0 - sin(tilt));
        pose.pose.position.z    = imv_dist_ * sin(tilt);
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        validCB(pose);

    }
}

UnlockDoor::SM::Handle UnlockDoor::stateWait()
{
    ROS_DEBUG("In STATE_WAIT");

    // Revert to neutral look.
    cur_pose_.header.frame_id    = robot_frame_;
    cur_pose_.pose.position.x    = look_x_;
    cur_pose_.pose.position.y    = look_y_;
    cur_pose_.pose.position.z    = look_z_;
    cur_pose_.pose.orientation.x = 0;
    cur_pose_.pose.orientation.y = 0;
    cur_pose_.pose.orientation.z = 0;
    cur_pose_.pose.orientation.w = 1;

    arm_ctrl_->setImpedance(0.0, 0.0);

    return STATE_WAIT;
}

UnlockDoor::SM::Handle UnlockDoor::stateSeek()
{
    ROS_DEBUG("In STATE_SEEK");
    arm_point_at_->generate(cur_pose_, arm_traj_);
    // Impedance is set in the timer loop to make sure the controller has a set
    // point first.

    return STATE_SEEK;
}

UnlockDoor::SM::Handle UnlockDoor::stateDone()
{
    ROS_DEBUG("In STATE_DONE");
    std_msgs::Empty msg;
    pub_done_.publish(msg);

    return STATE_WAIT;
}

void UnlockDoor::timerCB(const ros::TimerEvent&)
{
    const ros::Time now = ros::Time::now();
    if (!timeout_start_.isZero() && ((now - timeout_start_) > timeout_)) {
        timeout_start_ = ros::Time(); // Reset to zero.
        sm_.pushEvent(EVENT_TIMEOUT);
    }

    sm_.processQueue();

    pub_look_at_.publish(cur_pose_);

    if (sm_.state() == STATE_SEEK) {
        produceVel();
        tf::Point goal;
        arm_traj_.calc(ros::Time::now(), goal);
        arm_ctrl_->setPointInstant(goal);
        arm_ctrl_->setImpedance(imp_k_, 0.0);
    }

}

void UnlockDoor::produceVel()
{
    // 1. Transpose the location of the cardreader in the robot's XY plane.
    tf::StampedTransform f2r;
    try {
        tf_.lookupTransform(robot_frame_, fixed_frame_, ros::Time(), f2r);
    } catch (tf::TransformException e) {
        ROS_ERROR_THROTTLE(1.0,
                           "Cannot transform cardreader's pose "
                           "in the robot's frame: %s",
                           e.what());
        return;
    }

    tf::Stamped<tf::Pose> pose;
    tf::poseStampedMsgToTF(cur_pose_, pose);
    tf::Point p = (f2r * pose).getOrigin(); // The cardreader's position.

    double ex = p.x() - arm_dist_;
    double ey = p.y() - offset_y_;

    ROS_DEBUG_THROTTLE(1.0, 
                       "Velocity control (ex, ey): (%f, %f).",
                       ex,
                       ey);

    geometry_msgs::Twist cmd;

    cmd.linear.x  = std::max(std::min(lin_k_ * ex, lin_max_),       0.0);
    cmd.angular.z = std::max(std::min(ang_k_ * ey, ang_max_), -ang_max_); 

    pub_cmd_vel_.publish(cmd);

}

