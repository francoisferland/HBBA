#include <iw/events_generator.hpp>
#include <algorithm>

using namespace iw;

EventsGenerator::EventsGenerator(ros::NodeHandle& n, ros::NodeHandle& np)
{
    sub_desires_ = n.subscribe("desires_set", 1, 
        &EventsGenerator::desiresCB, this);
    sub_intention_ = n.subscribe("intention", 1,
        &EventsGenerator::intentionCB, this);
    sub_exploitation_ = n.subscribe("exploitation_match", 100,
        &EventsGenerator::exploitationCB, this);

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
        if (std::find(ids.begin(), ids.end(), id) == ids.end())
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
            map_[id] = FLAG_NONE;
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

        int& f = d->second;
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

void EventsGenerator::exploitationCB(const std_msgs::String::ConstPtr& msg)
{
    const std::string& id = msg->data;

    DesMap::iterator d = map_.find(id);
    if (d == map_.end())
    {
        // Shouldn't happen, as in intention callback.
        // Once again, skip and only warn in debug:
        ROS_DEBUG("Unknown desire being exploited: %s", id.c_str());
        return;
    }

    int& f = d->second;
    if (!(f & FLAG_EXP))
    {
        event(id, hbba_msgs::Event::EXP_ON);
        f |= FLAG_EXP;
    }

    // Removing FLAG_EXP should happen in two cases:
    //  - timeout,
    //  - replaced by another priority.
    // This means the exploitation matcher should be better coupled with the
    // events generator.
    // TODO: Use exploitation matchers to detect exploitation stops.

}

void EventsGenerator::event(const std::string& id, const unsigned char type)
{
    hbba_msgs::Event msg;
    msg.desire = id;
    msg.type = type;
    pub_events_.publish(msg);
}

