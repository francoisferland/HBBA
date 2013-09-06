#include <iw_translator/solver_model.hpp>

using namespace iw_translator;

namespace {
    /// \brief Return the index of an id in a indices map, or create it if it
    /// doesn't exist yet.
    int mapIndex(IndicesMap& map, const std::string& id) 
    {
        IndicesMap::left_map::const_iterator i = map.left.find(id);
        if (i == map.left.end()) {
            int v = map.size();
            map.left.insert(std::make_pair(id, v));
            return v;
        } else {
            return i->second;
        }
    }

    /// \brief Return a string identifier based on an index, or NULL_ID if it's
    /// not valid.
    const std::string& mapId(IndicesMap& map, const int i) 
    {
        static std::string null_id("NULL_ID");

        IndicesMap::right_map::const_iterator t = map.right.find(i);
        if (t != map.right.end()) {
            return t->second;
        } else {
            return null_id;
        }
    }
}

SolverModel::SolverModel(
    const std::vector<hbba_msgs::Strategy>&      strats,
    const std::vector<hbba_msgs::ResourceUsage>& caps)
{
    typedef std::vector<hbba_msgs::Strategy>::const_iterator      SIt;
    typedef std::vector<hbba_msgs::ResourceUsage>::const_iterator RIt;
    typedef Vector::const_iterator                                VIt;

    // First, build the indices maps, then the matrices can be properly sized.
    
    ROS_INFO("Strats count: %lu", strats.size());
    for (SIt i = strats.begin(); i != strats.end(); ++i) {
        ROS_INFO("Strats costs size: %lu", i->cost.size());
        mapIndex(strat_map_, i->id);
        mapIndex(cls_map_,   i->utility.id);
        for (RIt j = i->cost.begin(); j != i->cost.end(); ++j) {
            ROS_INFO("Resource info: %s", j->id.c_str());
            mapIndex(res_map_, j->id);
        }
        for (RIt j = i->utility_min.begin(); j != i->utility_min.end(); ++j) {
            mapIndex(cls_map_, j->id);
        }
    }

    for (RIt i = caps.begin(); i != caps.end(); ++i) {
        mapIndex(res_map_, i->id);
    }

    // Build the three strategies-related matrices:

    u_.resize(boost::extents[strat_map_.size()][cls_map_.size()]);
    c_.resize(boost::extents[strat_map_.size()][res_map_.size()]);
    r_.resize(boost::extents[strat_map_.size()][cls_map_.size()]);
    for (SIt i = strats.begin(); i != strats.end(); ++i) {
        int strat_i = mapIndex(strat_map_, i->id);
        u_[strat_i][mapIndex(cls_map_, i->utility.id)] = i->utility.value;
        for (RIt j = i->cost.begin(); j != i->cost.end(); ++j) {
            c_[strat_i][mapIndex(res_map_, j->id)] = j->value;
        }
        for (RIt j = i->utility_min.begin(); j != i->utility_min.end(); ++j) {
            r_[strat_i][mapIndex(cls_map_, j->id)] = j->value;
        }
    }

    // Fill the resource caps vector and look for inconsistencies:
    m_ = Vector(res_map_.size(), -1);
    for (RIt i = caps.begin(); i != caps.end(); ++i) {
        m_[mapIndex(res_map_, i->id)] = i->value;
    }
    for (size_t i = 0; i < m_.size(); ++i) {
        if (m_[i] < 0.0) {
            ROS_WARN(
                "Resource '%s' does not have a declared capacity.",
                mapId(res_map_, i).c_str());
        }
    }

}

