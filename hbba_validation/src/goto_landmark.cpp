#include <hbba_validation/goto_landmark.hpp>
#include <irl1_interaction/common.hpp>

using namespace hbba_validation;

GoToLandmark::GoToLandmark(ros::NodeHandle& n, ros::NodeHandle& np):
    obs_(n, np),
    desires_set_(desires_req_.request.desires)
{
    sub_goal_ = n.subscribe("landmark_goal", 10, &GoToLandmark::goalCB, this);

    scl_add_ = n.serviceClient<hbba_msgs::AddDesires>("add_desires", true);
}

void GoToLandmark::goalCB(const std_msgs::String& msg)
{
    if (!goTo(msg.data)) {
        ROS_ERROR("Received an unknown or invalid landmark on goal topic: %s", 
                  msg.data.c_str());
    }
}

bool GoToLandmark::goTo(const std::string& code)
{
    if (!obs_.hasLandmark(code)) {
        return false;
    }

    const geometry_msgs::PoseStamped& goal = 
        obs_.landmarks().find(code)->second;

    if (goal.header.stamp.isZero()) {
        return false;
    }

    ROS_DEBUG("Received a valid landmark goal (%s).", code.c_str());
    gotoDesire(goal);

    return true;
}

void GoToLandmark::sayDesire(const std::string& data)
{
    hbba_msgs::AddDesires req;
    std::vector<hbba_msgs::Desire>& desires = req.request.desires;
    desires.resize(1);
    hbba_msgs::Desire& des_say = desires[0];
    
    des_say.id        = ros::this_node::getName() + 
                        std::string("_goto_landmark_say");
    des_say.type      = "Say";
    des_say.intensity = 1;
    des_say.utility   = 1;

    irl1_interaction::stringToDialog(data, des_say.params);

    scl_add_.call(req);
}

void GoToLandmark::gotoDesire(const geometry_msgs::PoseStamped& goal)
{
    hbba_msgs::AddDesires req;
    std::vector<hbba_msgs::Desire>& desires = req.request.desires;
    desires.resize(1);
    hbba_msgs::Desire& des_goto = desires[0];
    
    des_goto.id        = ros::this_node::getName() + 
                         std::string("_goto_landmark_nav");
    des_goto.type      = "GoTo";
    des_goto.intensity = 1;
    des_goto.utility   = 1;

    irl1_interaction::poseStampedToNavGoal(goal, des_goto.params);

    scl_add_.call(req);
}

