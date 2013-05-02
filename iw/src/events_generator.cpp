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

    pub_events_ = n.advertise<hbba_msgs::Event>("events", 100);
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
    typedef std::vector<hbba_msgs::Desire> DesVec;
    const DesVec& desires = msg->desires;
    StrVec ids;
    ids.reserve(desires.size());
    for (DesVec::const_iterator d = desires.begin(); d != desires.end(); ++d)
    {
        ids.push_back(d->id);
    }

    // First, remove desires not in it the current desires set.
    StrVec del;
    for (DesMap::const_iterator i = map_.begin(); i != map_.end(); ++i)
    {
        const std::string& id = i->first;
        if (std::find(ids.begin(), ids.end(), id) != ids.end())
            del.push_back(id);
    }
    for (StrVec::const_iterator i = del.begin(); i != del.end(); ++i)
    {
        const std::string& id = *i;
        map_.erase(id);
        event(id, hbba_msgs::Event::DES_OFF);
    }

    // Then, generate events for new desires.
    for (StrVec::const_iterator i = ids.begin(); i != ids.end(); ++i)
    {
        const std::string& id = *i;
        if (map_.find(id) == map_.end())
        {
            map_[id].flags = FLAG_NONE;
            event(id, hbba_msgs::Event::DES_ON);
        }
    }

}

void EventsGenerator::intentionCB(const hbba_msgs::Intention::ConstPtr& msg)
{
    typedef std::vector<std::string> StrVec;
    // ROS bool vector are actually single bytes:
    typedef std::vector<unsigned char> StateVec; 
    const StrVec& ids = msg->desires;
    const StateVec& states = msg->enabled;

    assert(ids.size() == states.size());

    for (size_t i = 0; i < ids.size(); ++i)
    {
        const std::string& id = ids[i];
        const bool& state = states[i];
        DesMap::iterator d = map_.find(id);
        if (d == map_.end())
        {
            // This shouldn't happen. Skip, only warn in debug:
            ROS_DEBUG("Unknown desire in intention set: %s", id.c_str());
            continue;
        }

        int& f = d->second.flags;
        if (state && !(f & FLAG_INT))
        {
            f |= FLAG_INT;
            event(id, hbba_msgs::Event::INT_ON);
        } else if (!state && (f & FLAG_INT))
        {
            f &= (f | ~FLAG_INT);
            event(id, hbba_msgs::Event::INT_OFF);
        }
    }
}

void EventsGenerator::exploitationCB(const std::string& id)
{
    DesMap::iterator d = map_.find(id);
    if (d == map_.end())
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
        event(id, hbba_msgs::Event::EXP_ON);
        f |= FLAG_EXP;
    }

    detectExpOff();
}

void EventsGenerator::detectExpOff()
{
    ros::Time timeout_limit = ros::Time::now() - exp_timeout_;

    // Go through the list of desires.
    // If a desire's last exploitation time is over the timeout limit and its
    // exploitation flag is on, flip the flag and generate the event.
    DesMap::iterator d = map_.begin();
    while (d != map_.end())
    {
        const std::string& id = d->first;
        DesireData& data = (d++)->second;
        if (data.flags & FLAG_EXP && data.last_exp_ < timeout_limit)
        {
            data.flags &= ~FLAG_EXP;
            event(id, hbba_msgs::Event::EXP_OFF);
        }
    }
}

void EventsGenerator::event(const std::string& id, const unsigned char type)
{
    hbba_msgs::Event msg;
    msg.desire = id;
    msg.type = type;
    pub_events_.publish(msg);
}

bool EventsGenerator::cemCB(
    hbba_msgs::CreateExploitationMatcher::Request& req,
    hbba_msgs::CreateExploitationMatcher::Response& res)
{
    ros::NodeHandle n, nt(req.abtr_topic);
    ExploitationMatcher* em = new ExploitationMatcher(n, nt);
    matchers_.push_back(em);

    std::vector<hbba_msgs::ExploitationMatch>::const_iterator i = 
        req.matches.begin();
    while (i != req.matches.end())
        em->registerMatches(*(i++));

    em->registerMatchCB(&EventsGenerator::exploitationCB, this);

    return true;
}

