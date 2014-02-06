#include <hbba_validation/unlock_door.hpp>
#include <geometry_msgs/Twist.h>

using namespace hbba_validation;

UnlockDoor::UnlockDoor(ros::NodeHandle& n, ros::NodeHandle& np):
    localizer_openni_(n, ros::NodeHandle(np, "localizer_openni_")),
    localizer_imv_(n, ros::NodeHandle(np, "localizer_imv_"), true)
{
    np.param("fixed_frame", fixed_frame_, std::string("/odom"));
    np.param("robot_frame", robot_frame_, std::string("/base_link"));
    np.param("offset_y",    offset_y_,    0.25);
    np.param("look_x",      look_x_,      2.00);
    np.param("look_y",      look_y_,      0.00);
    np.param("look_z",      look_z_,      1.00);
    np.param("imp_k",       imp_k_,       35.0);
    np.param("arm_dist",    arm_dist_,    0.50);
    np.param("arm_vel",     arm_vel_,     0.25);

    localizer_openni_.registerValidCB(&UnlockDoor::validCB, this);
    localizer_openni_.registerInvalidCB(&UnlockDoor::invalidCB, this);
    localizer_imv_.registerCB(&UnlockDoor::greenCB, this);


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
    SM::Transitions transitions;
    SM::generateTransitionsMatrix(STATE_SIZE, EVENT_SIZE, transitions);
    transitions[STATE_WAIT][EVENT_VALID]   = STATE_SEEK;
    transitions[STATE_SEEK][EVENT_GREEN]   = STATE_WAIT;
    transitions[STATE_SEEK][EVENT_TIMEOUT] = STATE_WAIT;
    sm_ = SM(states, transitions, this);

    double p;
    np.param("period", p, 0.10);
    timer_ = n.createTimer(ros::Duration(p), &UnlockDoor::timerCB, this);

    pub_cmd_vel_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    pub_look_at_ = n.advertise<geometry_msgs::PoseStamped>("look_at_pose", 1);
}

void UnlockDoor::validCB(const geometry_msgs::PoseStamped& pose)
{
    // Save the detected pose in the fixed frame.
    try {
        tf::Stamped<tf::Pose> pose_tf;
        tf::poseStampedMsgToTF(pose, pose_tf);
        tf::StampedTransform t;
        tf_.lookupTransform(fixed_frame_, robot_frame_, ros::Time(), t);
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
    sm_.pushEvent(EVENT_INVALID);
}

void UnlockDoor::greenCB(const sensor_msgs::Image&, double, double)
{
    sm_.pushEvent(EVENT_GREEN);
}

UnlockDoor::SM::Handle UnlockDoor::stateWait()
{
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
    arm_point_at_->generate(cur_pose_, arm_traj_);
    // Impedance is set in the timer loop to make sure the controller has a set
    // point first.

    return STATE_SEEK;
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
        tf::Point goal;
        arm_traj_.calc(ros::Time::now(), goal);
        arm_ctrl_->setPointInstant(goal);
        arm_ctrl_->setImpedance(imp_k_, 0.0);
    }

}

