#include <iw_translator/iw_translator.hpp>
#include <iw_translator/strategy_parser.hpp>

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

    initStrategies();
}

void IWTranslator::desiresCB(const hbba_msgs::DesiresSet::ConstPtr& msg)
{
    Vector g, s;

    if (!solver_model_->convertDesires(*msg, g, s)) {
        ROS_WARN("Class with unknown desires will be ignored.");
    }

    Solver solver(*solver_model_, g, s);

    ActivationVector a;
    if (solver.result(a)) {
        ROS_DEBUG("Solving succeeded.");
        hbba_msgs::Intention out;
        for (size_t i = 0; i < a.size(); ++i) {
            const hbba_msgs::Strategy& strat = strats_[i];

            ROS_DEBUG(
                "Strategy %s activation: %s", 
                strats_[i].id.c_str(), 
                a[i] ? "true":"false");

            const std::string&         strat_id   = strat.id;
            const hbba_msgs::Desire*   desire     = a[i] ? 
                desireFromType(*msg, strat.utility.id) : NULL;
            const std::string&         des_type   = a[i] ?
                strats_[i].utility.id                  : EMPTY;
            const std::string&         des_id     = a[i] && desire ? 
                desire->id                             : EMPTY;
            const std::string&         des_params = a[i] && desire ?
                desire->params                         : EMPTY;        

            out.strategies.push_back(   strat_id);
            out.desires.push_back(      des_id);
            out.desire_types.push_back( des_type);
            out.params.push_back(       des_params);
            out.enabled.push_back(      a[i]);
        }

        out.stamp = ros::Time::now();
        pub_intention_.publish(out);

        activateIntention(out);
    };
}

void IWTranslator::initStrategies()
{
    ROS_INFO("Evaluating strategy script source...");
    std::stringstream ss;

    typedef std::vector<hbba_msgs::Strategy>::const_iterator It;
    for (It i = strats_.begin(); i != strats_.end(); ++i) {
        ss << i->source << std::endl;
    }

    std::string result;
    script_engine_.eval(ss.str(), result);
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
    assert(intent.enabled.size() == strats_.size());

    const std::vector<unsigned char>& a = intent.enabled;
    const std::vector<std::string>&   p = intent.params;

    std::stringstream ss;
    for (size_t i = 0; i < intent.enabled.size(); ++i) {
        const hbba_msgs::Strategy& strat = strats_[i];
        if (a[i] && strat.bringup_function != EMPTY) {
            ss << strats_[i].bringup_function << "(" << p[i] << ");";
        } else if (strats_[i].bringdown_function != EMPTY) {
            ss << strats_[i].bringdown_function << "();";
        }
    }

    std::string result;
    script_engine_.eval(ss.str(), result);

}

