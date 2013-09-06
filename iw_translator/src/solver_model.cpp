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
    const std::string& mapId(const IndicesMap& map, const int i) 
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
    
    for (SIt i = strats.begin(); i != strats.end(); ++i) {
        mapIndex(strat_map_, i->id);
        mapIndex(cls_map_,   i->utility.id);
        for (RIt j = i->cost.begin(); j != i->cost.end(); ++j) {
            mapIndex(res_map_, j->id);
        }
        for (RIt k = i->utility_min.begin(); k != i->utility_min.end(); ++k) {
            mapIndex(cls_map_, k->id);
        }
    }

    for (RIt j = caps.begin(); j != caps.end(); ++j) {
        mapIndex(res_map_, j->id);
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
        for (RIt k = i->utility_min.begin(); k != i->utility_min.end(); ++k) {
            r_[strat_i][mapIndex(cls_map_, k->id)] = k->value;
        }
    }

    // Sanity check: warn if a desire class has no utility producers:
    for (size_t k = 0; k < cls_map_.size(); ++k) {
        Scalar u_sum = 0.0;
        for (size_t i = 0; i < strat_map_.size(); ++i) {
            u_sum += u_[i][k];
        }

        if (u_sum == 0.0) {
            ROS_WARN(
                "Class '%s' has no producers.", 
                mapId(cls_map_, k).c_str());
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

std::string SolverModel::uAsCSV() const
{
    return matrixAsCSV(u_, cls_map_);
}

std::string SolverModel::cAsCSV() const
{
    return matrixAsCSV(c_, res_map_);
}

std::string SolverModel::rAsCSV() const
{
    return matrixAsCSV(r_, cls_map_);
}

std::string SolverModel::matrixAsCSV(
    const Matrix& mtx, 
    const IndicesMap& map) const 
{
    std::stringstream ss;

    // Header: Strategy | Resource 1 | Resource 2 | ... | Resource J 
    ss << "Strategy,";
    for (size_t i = 0; i < (map.size() - 1); ++i) {
        ss << mapId(map, i) << ",";
    }
    ss << mapId(map, map.size() - 1) << std::endl;

    for (size_t i = 0; i < strat_map_.size(); ++i) {
        ss << mapId(strat_map_, i) << ",";
        for (size_t j = 0; j < (map.size() - 1); ++j) {
            ss << mtx[i][j] << ",";
        }
        ss << mtx[i][map.size() - 1] << std::endl;
    }

    return ss.str();
}


