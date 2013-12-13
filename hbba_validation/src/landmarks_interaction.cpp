#include <hbba_validation/landmarks_interaction.hpp>
#include <boost/algorithm/string.hpp>

using namespace hbba_validation;

LandmarksInteraction::LandmarksInteraction(ros::NodeHandle& n, 
                                           ros::NodeHandle& np):
    goto_(n, np)
{
    sub_sphinx_ = n.subscribe("sphinx_result", 
                              10, 
                              &LandmarksInteraction::sphinxCB,
                              this);

    // State machine preparation:
    SM::States states;
    states.push_back(&LandmarksInteraction::stateWait);
    states.push_back(&LandmarksInteraction::stateSave);
    states.push_back(&LandmarksInteraction::stateLead);
    states.push_back(&LandmarksInteraction::stateGoTo);

    SM::Transitions transitions;
    SM::generateTransitionsMatrix(STATE_SIZE, EVENT_SIZE, transitions);
    transitions[STATE_WAIT][EVENT_REQ_SAVE] = STATE_SAVE;
    transitions[STATE_WAIT][EVENT_REQ_GOTO] = STATE_GOTO;

    sm_ = SM(states, transitions, this);
}

LandmarksInteraction::SM::Handle LandmarksInteraction::stateWait()
{
    ROS_DEBUG("In STATE_WAIT");
    return sm_.state();
}

LandmarksInteraction::SM::Handle LandmarksInteraction::stateSave()
{
    ROS_DEBUG("In STATE_SAVE");
    goto_.observer().saveLandmark(last_code_);
    return STATE_WAIT;
}

LandmarksInteraction::SM::Handle LandmarksInteraction::stateLead()
{
    ROS_DEBUG("In STATE_LEAD");
    // TODO: Interaction / say.
    return sm_.state();
}

LandmarksInteraction::SM::Handle LandmarksInteraction::stateGoTo()
{
    ROS_DEBUG("In STATE_GOTO");
    // TODO: Interaction / say when the code is valid/invalid.
    if (goto_.goTo(last_code_)) {
        ROS_DEBUG("STATE_GOTO: nav goal produced.");
    } else {
        ROS_DEBUG("STATE_GOTO: Could not produce a nav goal from '%s'",
                  last_code_.c_str());
    }
    return STATE_WAIT;
}

void LandmarksInteraction::pushEvent(const SM::Handle event)
{
    sm_.pushEvent(event);
    sm_.processQueue();
}

void LandmarksInteraction::sphinxCB(const std_msgs::String& msg)
{
    static const std::string req_save("this is landmark");
    static const std::string req_goto("goto landmark");

    std::string data = msg.data;
    boost::to_lower(data);
    
    boost::replace_all(data, "go to", "goto");

    boost::iterator_range<std::string::const_iterator> i;
    if (i = boost::find_first(data, req_save)) {
        if (codeFromRequest(data, i, last_code_)) {
            ROS_DEBUG("Got a valid request to save a landmark as '%s'", 
                      last_code_.c_str());
            pushEvent(EVENT_REQ_SAVE);
        } else {
            ROS_DEBUG("Invalid request to save a landmark: '%s'",
                      data.c_str());
        }
    } else if (i = boost::find_first(data, req_goto)) {
        if (codeFromRequest(data, i, last_code_)) {
            ROS_DEBUG("Got a valid request to go to landmark '%s'",
                      last_code_.c_str());
            pushEvent(EVENT_REQ_GOTO);
        } else {
            ROS_DEBUG("Invalid go to request: '%s'",
                      data.c_str());
        }
    } else {
        ROS_DEBUG("Unknown request: '%s'", data.c_str());
    }
}

bool LandmarksInteraction::codeFromRequest(const std::string& req, 
                                           const StringRange& loc,
                                           std::string&       out) const
{
    std::string::const_iterator l = (loc.end() + 1);
    if (l <= req.end()) {
        out = formatLandmark(std::string(l, l+1));
        return true;
    } else {
        return false;
    }
}

std::string LandmarksInteraction::formatLandmark(const std::string& code) const
{
    static const std::string landmark_prefix("IRL1_LANDMARK_");

    return landmark_prefix + code;
}

