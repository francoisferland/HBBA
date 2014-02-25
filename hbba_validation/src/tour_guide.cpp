#include <hbba_validation/tour_guide.hpp>

using namespace hbba_validation;

namespace {
    static const char* DESIRE_GOTO       = "GoTo";
    static const char* DESIRE_TURNAROUND = "TurnAround";
    static const char* DESIRE_SAY        = "Say";
    static const char* DESIRE_POINT_AT   = "PointAt";
    static const char* DESIRE_UNLOCK     = "UnlockDoor";

}

TourGuide::TourGuide(ros::NodeHandle& n, ros::NodeHandle& np)
{
    if (!parseScenario(np)) { 
        ROS_ERROR("Could not parse tour scenario - see previous errors for "
                  "details. The tour state machine will not be started.");
        return;
    }

    ROS_INFO("Tour correctly parsed, launching scenario...");

    sub_event_ = n.subscribe("events", 50, &TourGuide::eventCB, this);
}

TourGuide::~TourGuide()
{
    typedef std::vector<TourStep*>::iterator It;
    for (It i = steps_.begin(); i != steps_.end(); ++i) {
        delete *i;
    }
}

bool TourGuide::parseScenario(const ros::NodeHandle& np)
{
    if (!np.hasParam("tour")) {
        ROS_ERROR("Cannot find 'tour' element in given private namespace.");
        return false;
    }

    XmlRpc::XmlRpcValue tour_ns;
    np.getParam("tour", tour_ns);

    if (tour_ns.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_ERROR("'tour' is not a struct.");
        return false;
    }

    if (!tour_ns.hasMember("start")) {
        ROS_ERROR("The scenario description does not specify a start state.");
        return false;
    }

    std::string                         start_state;
    std::map<std::string, unsigned int> indices;
    std::vector<std::string>            links;
    std::vector<unsigned int>           types;

    typedef XmlRpc::XmlRpcValue::iterator It;
    bool has_errors = false;
    for (It i = tour_ns.begin(); i != tour_ns.end(); ++i) {
        const std::string&   key  = i->first;
        XmlRpc::XmlRpcValue& elem = i->second;
        std::string          next;              // ID of next step.
        unsigned int         type;              // Type of switching event.

        if (key == "start") {
            if (elem.getType() != XmlRpc::XmlRpcValue::TypeString) {
                ROS_ERROR("'start' element is not a string.");
                has_errors = true;
            }
            start_state = std::string(elem);
        } else if (parseElem(key, elem, next, type)) {
            indices[key] = steps_.size() - 1;
            links.push_back(next);
            types.push_back(type);
        } else {
            has_errors = true;
        }
    }

    if (has_errors) {
        return false;
    } 

    TourGuideStateMachine::Transitions transitions;
    TourGuideStateMachine::generateTransitionsMatrix(steps_.size(), 
                                                     EVENT_SIZE, 
                                                     transitions);
    TourGuideStateMachine::States states;
    for (unsigned int i = 0; i < steps_.size(); ++i) {
        states.push_back(&TourGuide::step);
        transitions[i][types[i]] = indices[links[i]];
    }

    sm_ = SM(states, transitions, this);

    return true;
}

bool TourGuide::parseElem(const std::string&         key, 
                                XmlRpc::XmlRpcValue& elem,
                                std::string&         next,
                                unsigned int&        type)
{
    if (!elem.hasMember("type")) {
        ROS_ERROR("No type defined for element '%s'.", key.c_str()); 
        return false;
    }

    if (!elem.hasMember("next")) {
        ROS_ERROR("No next step defined for element '%s'.", key.c_str());
        return false;
    }
    next = std::string(elem["next"]);

    XmlRpc::XmlRpcValue type_v = elem["type"];
    if (type_v.getType() != XmlRpc::XmlRpcValue::TypeString) {
        ROS_ERROR("Type definition for '%s' is not a string.", key.c_str());
        return false;
    }

    std::string type_s(type_v);
    TourStep* step = 0;
    if (type_s == "waypoint") {
        step = new WaypointStep(key);
        type = EVENT_LOC_REACHED;
    } else if (type_s == "speech") {
        SpeechStep* ss = new SpeechStep(key);
        step = ss;
        type = EVENT_SPEECH_DONE;

        if (elem.hasMember("text")) {
            ss->text(elem["text"]);
        } else {
            ROS_WARN("Speech step '%s' has no 'text' element.", key.c_str());
        }

        if (elem.hasMember("point_at")) {
            std::string pd(elem["point_at"]);
            if (pd == "left") {
                ss->pointAt(SpeechStep::POINT_LEFT);
            } else if (pd == "right") {
                ss->pointAt(SpeechStep::POINT_RIGHT);
            } else {
                ROS_WARN("Speech step '%s' has unknown point_at direction: "
                         "'%s'.",
                         key.c_str(),
                         pd.c_str());
            }
        }
    } else if (type_s == "unlock") {
        step = new UnlockStep(key);
        type = EVENT_UNLOCKED;
    } else if (type_s == "turnaround") {
        step = new TurnaroundStep(key);
        type = EVENT_TURNAROUND_DONE;
    }

    if (step == 0) {
        ROS_ERROR("Unknown type '%s' for step '%s'.", 
                  type_s.c_str(),
                  key.c_str());
        return false;
    }

    steps_.push_back(step);

    return true;
}

void TourGuide::eventCB(const hbba_msgs::Event& msg)
{
    if (std::find(cur_desire_ids_.begin(), 
                  cur_desire_ids_.end(), 
                  msg.desire) == cur_desire_ids_.end()) {
        return;
    }

    if (msg.type == hbba_msgs::Event::ACC_ON) {
        if (msg.desire_type == DESIRE_GOTO) {        
            ROS_DEBUG("Pushing event LOC_REACHED");
            sm_.pushEvent(EVENT_LOC_REACHED);
        } else if (msg.desire_type == DESIRE_UNLOCK) {
            ROS_DEBUG("Pushing event UNLOCKED");
            sm_.pushEvent(EVENT_UNLOCKED);
        } else if (msg.desire_type == DESIRE_TURNAROUND) {
            ROS_DEBUG("Pushing event TURNAROUND_DONE");
            sm_.pushEvent(EVENT_TURNAROUND_DONE);
        } else if (msg.desire_type == DESIRE_SAY) {
            ROS_DEBUG("Pushing event SPEECH_DONE");
            sm_.pushEvent(EVENT_SPEECH_DONE);
        }
    }
}

TourGuide::SM::Handle TourGuide::step()
{
    std::vector<hbba_msgs::Desire> desires_set;

    TourStep* state = steps_[sm_.state()];
    state->run(desires_set);

    // TODO: Call remove_desires on old set, add_desires on new one.
    
    // Save the current active ids for future removal, event detection:
    cur_desire_ids_.clear();
    typedef std::vector<hbba_msgs::Desire>::const_iterator It;
    for (It i = desires_set.begin(); i != desires_set.end(); ++i) {
        cur_desire_ids_.push_back(i->id);
    }

    // TODO: Do proper transitions when possible.
    return sm_.state();

}

bool WaypointStep::run(std::vector<hbba_msgs::Desire>& desires_set)
{
    desires_set.resize(1);
    hbba_msgs::Desire& d = desires_set[0];

    d.id   = "TourGuide" + name() + DESIRE_GOTO;
    d.type = DESIRE_GOTO;
    // TODO: Pose.
    return false;
}

bool SpeechStep::run(std::vector<hbba_msgs::Desire>& desires_set)
{
    hbba_msgs::Desire ds;
    ds.id   = "TourGuide" + name() + DESIRE_SAY;
    ds.type = DESIRE_SAY;
    // TODO: Text.

    desires_set.push_back(ds);

    if (point_at_ != POINT_NONE) {
        hbba_msgs::Desire dp;
        dp.id   = "TourGuide" + name() + DESIRE_POINT_AT;
        dp.type = DESIRE_POINT_AT;
        // TODO: Direction.

        desires_set.push_back(dp);
    }

    return false;
}

bool UnlockStep::run(std::vector<hbba_msgs::Desire>& desires_set)
{
    desires_set.resize(1);
    hbba_msgs::Desire& d = desires_set[0];

    d.id   = "TourGuide" + name() + DESIRE_UNLOCK;
    d.type = DESIRE_UNLOCK;

    return false;
}

bool TurnaroundStep::run(std::vector<hbba_msgs::Desire>& desires_set)
{
    desires_set.resize(1);
    hbba_msgs::Desire& d = desires_set[0];

    d.id   = "TourGuide" + name() + DESIRE_TURNAROUND;
    d.type = DESIRE_TURNAROUND;
    return false;
}

