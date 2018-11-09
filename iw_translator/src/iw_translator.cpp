#include <iw_translator/iw_translator.hpp>
#include <iw_translator/strategy_parser.hpp>
#include <hbba_msgs/IWTranslatorStatus.h>

using namespace iw_translator;

namespace 
{
    // Use an empty string for unknown/unmatched desires and other tests:
    static const std::string EMPTY("");

    /// \brief Find the first desire with the given type.
    ///
    /// \return NULL if the type could not be found.
    const hbba_msgs::Desire* desireFromType(
        const hbba_msgs::DesiresSet& desires_set,
        const std::string& type)
    {
        typedef std::vector<hbba_msgs::Desire> Desires;
        typedef Desires::const_iterator It;
        const Desires& desires = desires_set.desires;
        for (It i = desires.begin(); i != desires.end(); ++i) {
            if (i->type == type) {
                return &(*i);
            }
        }

        return NULL;
    }

}

IWTranslator::IWTranslator(ros::NodeHandle& n, ros::NodeHandle& np)
{
    if (!np.hasParam("strategies")) {
        ROS_ERROR(
            "No strategies defined for IWTranslator, will not be initialized.");
        return;
    } 
    XmlRpc::XmlRpcValue strats_def;
    np.getParam("strategies", strats_def);
    if (!StrategyParser::parseArray(strats_def, strats_)) {
        ROS_ERROR("Cannot properly parse strategies, will not run.");
        return;
    }

    // Note: An empty resource caps vector is valid, just not as useful.
    std::vector<hbba_msgs::ResourceUsage> res_caps;
    if (np.hasParam("res_caps")) {
        XmlRpc::XmlRpcValue res_caps_def;
        np.getParam("res_caps", res_caps_def);
        StrategyParser::parseCosts(res_caps_def, res_caps);
    }

    np.param("max_p",      solver_params_.max_p,       true);
    np.param("solver_log", solver_params_.log,        false);
    np.param("time_limit", solver_params_.time_limit,     0);
    np.param("solver_sa",  solver_params_.sa,         false);

    solver_model_.reset(new SolverModel(strats_, res_caps));

    sub_desires_ = n.subscribe(
        "desires_set", 
        10, 
        &IWTranslator::desiresCB, 
        this);

    pub_intention_ = n.advertise<hbba_msgs::Intention>(
        "intention",
        10,
        true); // Always latch.
    pub_status_    = n.advertise<hbba_msgs::IWTranslatorStatus>(
        "iw_status",
        10,
        true); // Always latch.


    // We set the last activation state to all off, since we can assume that
    // strategies at initialization are already in a "off" state.
    last_a_ = std::vector<unsigned char>(strats_.size(), 0);
    last_p_ = std::vector<std::string>(strats_.size(), std::string("")); 

    std::string custom_script;
    np.param("custom_script", custom_script, std::string(""));
    evalScript(custom_script);
    initStrategies();
}

void IWTranslator::desiresCB(const hbba_msgs::DesiresSet::ConstPtr& msg)
{
    Vector g, s;

    if (!solver_model_->convertDesires(*msg, g, s)) {
        ROS_WARN("Desires with unknown classes will be ignored.");
    }

    Solver solver(*solver_model_, g, s, solver_params_);

    // Add current desire set to status message.
    hbba_msgs::IWTranslatorStatus status;
    status.stamp = ros::Time::now();
    status.desire_class.reserve(    msg->desires.size());
    status.desire_intensity.reserve(msg->desires.size());
    status.desire_utility.reserve(  msg->desires.size());
    typedef std::vector<hbba_msgs::Desire>::const_iterator It;
    for (It i = msg->desires.begin(); i != msg->desires.end(); ++i) {
        status.desire_class.push_back(    i->type);
        status.desire_intensity.push_back(i->intensity);
        status.desire_utility.push_back(  i->utility);
    }

    ActivationVector a;
    if (solver.result(a)) {
        ROS_DEBUG("Solving succeeded.");
        hbba_msgs::Intention out;
        out.stamp = ros::Time::now();

        for (size_t i = 0; i < a.size(); ++i) {
            const hbba_msgs::Strategy& strat = strats_[i];

            ROS_DEBUG(
                "Strategy %s activation: %s", 
                strats_[i].id.c_str(), 
                a[i] ? "true":"false");

            if (a[i]) {
                status.enabled_strategies.push_back(strat.id);
            }

            const std::string&         strat_id   = strat.id;
            const hbba_msgs::Desire*   desire     = a[i] ? 
                                                    desireFromType(
                                                        *msg,
                                                        strat.utility.id)
                                                  : NULL;
            const std::string&         des_type   = a[i] ?
                                                    strats_[i].utility.id
                                                  : EMPTY;
            const std::string&         des_id     = a[i] && desire ? 
                                                    desire->id
                                                  : EMPTY;
            const std::string&         des_params = a[i] && desire ?
                                                    desire->params
                                                  : EMPTY;        
            const int                  des_int    = a[i] && desire ?
                                                    desire->intensity
                                                  : -1;


            out.strategies.push_back(   strat_id);
            out.desires.push_back(      des_id);
            out.desire_types.push_back( des_type);
            out.params.push_back(       des_params);
            out.enabled.push_back(      a[i]);
        }

        pub_intention_.publish(out);

        activateIntention(out);
    };

    // Note that, unlike for intention, we still publish stats even if the 
    // solving process was not successful.
    pub_status_.publish(status);
}

void IWTranslator::initStrategies()
{
    ROS_INFO("Evaluating strategy script source...");
    std::stringstream ss;

    typedef std::vector<hbba_msgs::Strategy>::const_iterator It;
    for (It i = strats_.begin(); i != strats_.end(); ++i) {
        ss << i->source << std::endl;
    }

    evalScript(ss.str());
}

void IWTranslator::evalScript(const std::string& script)
{
    std::string result;
    script_engine_.eval(script, result);
    if (result == "undefined") {
        ROS_INFO("Evaluation done.");
    } else {
        ROS_WARN(
            "Evaluation done, see console for details. Result: '%s'",
            result.c_str());
    }
}

void IWTranslator::activateIntention(const hbba_msgs::Intention& intent)
{
    // Check for activation state transitions:
    //  - 0 -> 1: bup, covers params changes between common strats.
    //  - 1 -> 0: bdn.
    //  - 0 -> 0: nop.
    //  - 1 -> 1 & params change: bup.
    // NOTE: We assume all strategies are deactivated at startup.
    // Furthermore, always run bdn first, so that a bup won't be cancelled by a
    // bdn in a later call.
    
    assert(intent.enabled.size() == strats_.size());

    const std::vector<unsigned char>& a = intent.enabled;
    const std::vector<std::string>&   p = intent.params;

    std::stringstream ss_bup, ss_bdn;
    for (size_t i = 0; i < intent.enabled.size(); ++i) {
        const hbba_msgs::Strategy& strat = strats_[i];
        if (a[i] ^ last_a_[i] || (a[i] && p[i] != last_p_[i])) {
            if (a[i] && strat.bringup_function != EMPTY) {
                ss_bup << strats_[i].bringup_function << "(" << p[i] << ");";
            } else if (strats_[i].bringdown_function != EMPTY) {
                ss_bdn << strats_[i].bringdown_function << "();";
            }
        }
    }

    std::string script_full = ss_bdn.str() + ss_bup.str();
    std::string result;
    script_engine_.eval(script_full, result);

    last_a_ = a;
    last_p_ = p;

}

