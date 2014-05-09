#include <iw_translator/solver_model.hpp>
#include <ros/ros.h>

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

    /// \brief Return the index of an id in an indices map, but return -1 if
    /// it's unknown.
    int mapIndex(const IndicesMap& map, const std::string& id) 
    {
        IndicesMap::left_map::const_iterator i = map.left.find(id);
        if (i == map.left.end()) {
            return -1;
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

SolverModel::SolverModel()
{
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
    ur_.resize(boost::extents[strat_map_.size()][cls_map_.size()]);
    std::fill(u_.origin(),  u_.origin()  + u_.size(),  0.0);
    std::fill(c_.origin(),  c_.origin()  + c_.size(),  0.0);
    std::fill(r_.origin(),  r_.origin()  + r_.size(),  0.0);
    std::fill(ur_.origin(), ur_.origin() + ur_.size(), 0.0);

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

    // Combined utility matrix (U - R) generation:
    for (size_t i = 0; i < ur_.shape()[0]; ++i) {
        for (size_t k = 0; k < ur_.shape()[1]; ++k) {
            ur_[i][k] = u_[i][k] - r_[i][k];
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

bool SolverModel::convertDesires(const hbba_msgs::DesiresSet& desires_set, 
                                 Vector& g,
                                 Vector& s) const
{
    typedef const std::vector<hbba_msgs::Desire> DVec;
    typedef DVec::const_iterator DIt;

    bool ok = true;

    g.resize(cls_map_.size());
    s.resize(cls_map_.size());
    std::fill(g.begin(), g.end(), -1);
    std::fill(s.begin(), s.end(),  0);

    DVec desires = desires_set.desires;
    for (DIt i = desires.begin(); i != desires.end(); ++i) {
        const hbba_msgs::Desire& des = *i;

        int k = mapIndex(cls_map_, des.type);
        if (k < 0) {
            ROS_WARN("Unknown desire class in model: %s", des.type.c_str());
            ok = false;
        } else {
            g[k] = des.utility;
            s[k] = des.intensity;
        }
    }

    return ok;
}

const std::string& SolverModel::strategyId(int i) const
{
    return mapId(strat_map_, i);
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

std::string SolverModel::urAsCSV() const
{
    return matrixAsCSV(ur_, cls_map_);
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


