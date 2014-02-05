#include <hbba_validation/unlock_door.hpp>
#include <geometry_msgs/Twist.h>

using namespace hbba_validation;

UnlockDoor::UnlockDoor(ros::NodeHandle& n, ros::NodeHandle& np):
    localizer_(n, np)
{
    np.param("fixed_frame", fixed_frame_, std::string("/odom"));
    np.param("robot_frame", robot_frame_, std::string("/base_link"));
    np.param("offset_y",    offset_y_,    0.25);
    np.param("look_x",      look_x_,      2.00);
    np.param("look_y",      look_y_,      0.00);
    np.param("look_z",      look_z_,      1.00);
    np.param("imp_k",       imp_k_,       35.0);

    localizer_.registerValidCB(&UnlockDoor::validCB, this);
    localizer_.registerInvalidCB(&UnlockDoor::invalidCB, this);

    SM::States states;
    states.push_back(&UnlockDoor::stateWait);
    states.push_back(&UnlockDoor::stateSeek);
    SM::Transitions transitions;
    SM::generateTransitionsMatrix(STATE_SIZE, EVENT_SIZE, transitions);
    transitions[STATE_WAIT][EVENT_VALID]   = STATE_SEEK;
    transitions[STATE_SEEK][EVENT_INVALID] = STATE_WAIT;
    sm_ = SM(states, transitions, this);

    double p;
    np.param("period", p, 0.10);
    timer_ = n.createTimer(ros::Duration(p), &UnlockDoor::timerCB, this);

    pub_cmd_vel_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    pub_look_at_ = n.advertise<geometry_msgs::PoseStamped>("look_at_pose", 1);
}

void UnlockDoor::validCB(const geometry_msgs::PoseStamped& pose)
{
    // TODO: Transform the pose in the fixed frame.
    cur_pose_ = pose;
    sm_.pushEvent(EVENT_VALID);
}

void UnlockDoor::invalidCB()
{
    sm_.pushEvent(EVENT_INVALID);
}

UnlockDoor::SM::Handle UnlockDoor::stateWait()
{
    // TODO: Set impedance to zero, stop the robot from moving.

    return STATE_WAIT;
}

UnlockDoor::SM::Handle UnlockDoor::stateSeek()
{
    // TODO: Set impedance to imp_k.

    pub_look_at_.publish(cur_pose_);
    
    return STATE_SEEK;
}

void UnlockDoor::timerCB(const ros::TimerEvent&)
{
    sm_.processQueue();

    switch (sm_.state()) {
        case STATE_WAIT:
            break;
        case STATE_SEEK:
            break;
        default:
            break;
    };
}

