#include <hbba_validation/tour_guide.hpp>

using namespace hbba_validation;

TourGuide::TourGuide(ros::NodeHandle& n, ros::NodeHandle& np)
{
    if (!parseScenario(np)) { 
        ROS_ERROR("Could not parse tour scenario - see previous errors for "
                  "details. The tour state machine will not be started.");
        return;
    }

    ROS_INFO("Tour correctly parsed, launching scenario...");
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

    std::string start_state;

    typedef XmlRpc::XmlRpcValue::iterator It;
    bool has_errors = false;
    for (It i = tour_ns.begin(); i != tour_ns.end(); ++i) {
        const std::string&   key  = i->first;
        XmlRpc::XmlRpcValue& elem = i->second;

        if (key == "start") {
            if (elem.getType() != XmlRpc::XmlRpcValue::TypeString) {
                ROS_ERROR("'start' element is not a string.");
                has_errors = true;
            }
            start_state = std::string(elem);
        } else {
            has_errors |= !parseElem(key, elem);
        }
    }

    return has_errors;
}

bool TourGuide::parseElem(const std::string& key, XmlRpc::XmlRpcValue& elem)
{
    if (!elem.hasMember("type")) {
        ROS_ERROR("No type defined for element '%s'.", key.c_str()); 
        return false;
    }

    XmlRpc::XmlRpcValue type_v = elem["type"];
    if (type_v.getType() != XmlRpc::XmlRpcValue::TypeString) {
        ROS_ERROR("Type definition for '%s' is not a string.", key.c_str());
        return false;
    }

    std::string type(type_v);
    TourStep* step = 0;
    if (type == "waypoint") {
        step = new WaypointStep(key);
    } else if (type == "speech") {
        SpeechStep* ss = new SpeechStep(key);
        step = ss;

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
    }

    if (step == 0) {
        ROS_ERROR("Unknown type '%s' for step '%s'.", 
                  type.c_str(),
                  key.c_str());
        return false;
    }

    steps_.push_back(step);

    return true;
}

bool WaypointStep::run()
{
    return false;
}

bool SpeechStep::run()
{
    return false;
}

bool UnlockStep::run()
{
    return false;
}

bool TurnaroundStep::run()
{
    return false;
}

