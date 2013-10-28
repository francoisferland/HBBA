#include <iw/events_generator.hpp>
#include <algorithm>

using namespace iw;

EventsGenerator::EventsGenerator(ros::NodeHandle& n, ros::NodeHandle& np)
{
	double p;
	np.param("exp_timeout", p, 0.5);
	exp_timeout_ = ros::Duration(p);

	sub_desires_ = n.subscribe("desires_set", 1,
			&EventsGenerator::desiresCB, this);
	sub_intention_ = n.subscribe("intention", 1,
			&EventsGenerator::intentionCB, this);
	srv_cem_ = n.advertiseService("create_exploitation_matcher",
			&EventsGenerator::cemCB, this);
	srv_ctem_ = n.advertiseService("create_topic_exploitation_matcher",
			&EventsGenerator::ctemCB, this);

	pub_events_ = n.advertise<hbba_msgs::Event>("events", 100);

	ros::NodeHandle rosgraph_np(np, "rosgraph_monitor");
	rosgraph_monitor_.reset(new RosgraphMonitor(n, rosgraph_np));
	rosgraph_monitor_->registerCB(&EventsGenerator::rosgraphEventsCB, this);

	timer_ = n.createTimer(ros::Duration(1.0), &EventsGenerator::timerCB, this);

    parseExploitationMatches(n);
}

EventsGenerator::~EventsGenerator()
{
	std::list<ExploitationMatcher*>::iterator i = matchers_.begin();
	while (i != matchers_.end())
		delete *(i++);
}

void EventsGenerator::desiresCB(const hbba_msgs::DesiresSet::ConstPtr& msg)
{
	typedef std::vector<std::string> StrVec;
	typedef std::list<std::string> StrList;
	typedef std::vector<hbba_msgs::Desire> DesVec;
	typedef std::map<std::string,std::string> TypeMap;
	const DesVec& desires = msg->desires;
	TypeMap typeMap;
	StrVec ids;
	ids.reserve(desires.size());
	for (DesVec::const_iterator d = desires.begin(); d != desires.end(); ++d)
	{
		ids.push_back(d->id);
		typeMap[d->id] = d->type;
	}

    // First, remove desires not in it the current desires set.
    StrList del;
    for (Model::const_iterator i = model_.begin(); i != model_.end(); ++i)
    {
        const std::string& id = i->first;
        if (std::find(ids.begin(), ids.end(), id) == ids.end())
            del.push_back(id);
    }
    for (StrList::const_iterator i = del.begin(); i != del.end(); ++i)
    {
        const std::string& id = *i;
	
        event(id, model_[id].type, hbba_msgs::Event::DES_OFF);
        if (model_[id].flags & FLAG_INT)
            event(id, model_[id].type, hbba_msgs::Event::INT_OFF);
        if (model_[id].flags & FLAG_EXP)
            event(id, model_[id].type, hbba_msgs::Event::EXP_OFF);

        model_.erase(id);
    }

	// Then, generate events for new desires.
	for (StrVec::const_iterator i = ids.begin(); i != ids.end(); ++i)
	{
		const std::string& id = *i;
		if (model_.find(id) == model_.end())
		{
			model_[id].flags = FLAG_NONE;
			model_[id].type = typeMap[id]; //set type in the model
			event(id, model_[id].type, hbba_msgs::Event::DES_ON);
		}
	}

}

void EventsGenerator::intentionCB(const hbba_msgs::Intention::ConstPtr& msg)
{
	typedef std::vector<std::string> StrVec;
	// ROS bool vector are actually single bytes:
	typedef std::vector<unsigned char> StateVec;
	const StrVec&   ids     = msg->desires;
	const StrVec&   d_types = msg->desire_types;
	const StateVec& states  = msg->enabled;

	assert((ids.size() == states.size()) && (d_types.size() == states.size()));

	for (size_t i = 0; i < ids.size(); ++i)
	{
		const std::string& id     = ids[i];
		const std::string& d_type = d_types[i];
		if (id == "")
		{
			// Skip unmatched strategies.
			continue;
		}

		const bool& state = states[i];
		Model::iterator d = model_.find(id);
		if (d == model_.end())
		{
			// Add it to the desires' map.
			// This can happen if we receive the intention message before the
			// desires set update.
			model_[id].flags = FLAG_NONE;
		}

		int& f = d->second.flags;
		if (state && !(f & FLAG_INT))
		{
			f |= FLAG_INT;
			event(id, d_type, hbba_msgs::Event::INT_ON);
		} else if (!state && (f & FLAG_INT))
		{
			f &= ~FLAG_INT;
			event(id, d_type, hbba_msgs::Event::INT_OFF);
		}
	}
}

void EventsGenerator::exploitationCB(const std::string& id)
{
	Model::iterator d = model_.find(id);
	if (d == model_.end())
	{
		// Shouldn't happen, as in intention callback.
		// Once again, skip and only warn in debug:
		ROS_DEBUG("Unknown desire being exploited: %s", id.c_str());
		return;
	}

	DesireData& data = d->second;
	data.last_exp_ = ros::Time::now();
	int& f = data.flags;
	if (!(f & FLAG_EXP))
	{
		event(id, data.type, hbba_msgs::Event::EXP_ON);
		f |= FLAG_EXP;
	}

	detectExpOff();
}

void EventsGenerator::rosgraphEventsCB(
		const hbba_msgs::RosgraphEvents& msg)
{
	typedef std::vector<hbba_msgs::RosgraphEvent>::const_iterator EvIt;
	for (EvIt i = msg.events.begin(); i != msg.events.end(); ++i) {
		if (i->activity > 0) {
			typedef std::vector<std::string>::const_iterator StIt;
			TEMMap::const_iterator tem = tem_map_.find(i->topic_name);
			if (tem == tem_map_.end()) {
				ROS_DEBUG(
						"Unknown topic for matcher: %s",
						i->topic_name.c_str());
				continue;
			}
			for (StIt j = tem->second.begin(); j != tem->second.end(); ++j) {
				exploitationCB(*j);
			}
		}
	}
}

void EventsGenerator::detectExpOff()
{
	ros::Time timeout_limit = ros::Time::now() - exp_timeout_;

	// Go through the list of desires.
	// If a desire's last exploitation time is over the timeout limit and its
	// exploitation flag is on, flip the flag and generate the event.
	Model::iterator d = model_.begin();
	while (d != model_.end())
	{
		const std::string& id = d->first;
		DesireData& data = (d++)->second;
		if ((data.flags & FLAG_EXP) && (data.last_exp_ < timeout_limit))
		{
			data.flags &= ~FLAG_EXP;
			event(id, data.type, hbba_msgs::Event::EXP_OFF);
		}
	}
}

void EventsGenerator::event(
		const std::string& id,
		const std::string& d_type,
		const unsigned char type)
{
	hbba_msgs::Event msg;
	msg.desire      = id;
	msg.desire_type = d_type;
	msg.type        = type;
	pub_events_.publish(msg);

	ROS_DEBUG("Current events state:");
	Model::const_iterator d = model_.begin();
	while (d != model_.end())
	{
		const std::string& id = d->first;
		const DesireData& data = (d++)->second;
		ROS_DEBUG(" - %s: %i, last exp: %f",
				id.c_str(),
				data.flags,
				(ros::Time::now() - data.last_exp_).toSec());

	}
}

void EventsGenerator::cem(
    const std::string& topic_name, 
    const std::vector<hbba_msgs::ExploitationMatch>& matches)
{
	ros::NodeHandle n, nt(topic_name);

	ExploitationMatcher* em = new ExploitationMatcher(n, nt);
	matchers_.push_back(em);

	std::vector<hbba_msgs::ExploitationMatch>::const_iterator i =
			matches.begin();
	while (i != matches.end())
		em->registerMatches(*(i++));

	em->registerMatchCB(&EventsGenerator::exploitationCB, this);

	ROS_INFO("ExploitationMatcher created for %s", topic_name.c_str());

}

bool EventsGenerator::cemCB(
		hbba_msgs::CreateExploitationMatcher::Request& req,
		hbba_msgs::CreateExploitationMatcher::Response& res)
{
    cem(req.topic, req.matches);
	return true;
}

bool EventsGenerator::ctemCB(
		hbba_msgs::RegisterTopicExploitationMatches::Request& req,
		hbba_msgs::RegisterTopicExploitationMatches::Response& res)
{
	tem_map_[req.topic] = req.matches;
	return true;
}

void EventsGenerator::parseExploitationMatches(const ros::NodeHandle& n)
{
    if (!n.hasParam("exploitation_matches")) {
        return;
    }

    ROS_INFO("Parsing parameters for exploitation matches ...");

    XmlRpc::XmlRpcValue node;
    n.getParam("exploitation_matches", node);

    if (node.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("'exploitation_matches' is not an array, ignoring.");
        return;
    }

    for (int i = 0; i != node.size(); ++i) {
        XmlRpc::XmlRpcValue& topic_node  = node[i];

        if (!topic_node.hasMember("topic_name")) {
            ROS_ERROR("Topic node has no topic_name, skipping.");
            continue;
        }

        if (!topic_node.hasMember("matches")) {
            ROS_ERROR("Topic node has no matches, skipping.");
            continue;
        }

        const std::string&   topic_name   = topic_node["topic_name"];
        XmlRpc::XmlRpcValue& matches_node = topic_node["matches"];

        if (matches_node.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_ERROR("Matches for '%s' is not an array, skipping.",
                      topic_name.c_str());
            continue;
        }

        std::vector<hbba_msgs::ExploitationMatch> matches;
        matches.reserve(matches_node.size());
        for (int j = 0; j < matches_node.size(); ++j) {
            XmlRpc::XmlRpcValue& elem = matches_node[j];
            if (elem.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
                ROS_ERROR("Element for '%s' is not a proper struct, skipping.",
                          topic_name.c_str());
                continue;
            }

            if (!elem.hasMember("priority")) {
                ROS_ERROR("Element for '%s' does not contain a priority, "
                          "skipping.",
                          topic_name.c_str());
                continue;
            }

            int priority = static_cast<int>(elem["priority"]);

            if (!elem.hasMember("desire_type")) {
                ROS_ERROR("Element for priority %i of '%s' does not contain a "
                          "desire type, skipping.",
                          priority,
                          topic_name.c_str());
                continue;
            }

            XmlRpc::XmlRpcValue& desire_types = elem["desire_type"];
            if (desire_types.getType() != XmlRpc::XmlRpcValue::TypeArray) {
                ROS_ERROR("Desire type for priority %i of '%s' is not an "
                          "array, skipping.",
                          priority,
                          topic_name.c_str());
                continue;
            }

            hbba_msgs::ExploitationMatch match;
            match.priority = priority;
            match.classes.reserve(desire_types.size());
            for (int k = 0; k < desire_types.size(); ++k) {
                XmlRpc::XmlRpcValue& dtype = desire_types[k];
                if (dtype.getType() != XmlRpc::XmlRpcValue::TypeString) {
                    ROS_ERROR("Desire type in '%s' is not a string, skipping.",
                              topic_name.c_str());
                    continue;
                }

                match.classes.push_back(dtype);
            }

            matches.push_back(match);
        }

        if (matches.size()) {
            cem(topic_name, matches);
        }
    }
}

void EventsGenerator::timerCB(const ros::TimerEvent&)
{
	// Perform routine maintenance on data.

	// Exploitation timeouts have to be checked periodically.
	// A problem occurs if we stop receiving exploitation matches: timeouts will
	// be detected too late.
	detectExpOff();
}

