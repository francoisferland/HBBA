#include <iw_translator/strategy_parser.hpp>

using namespace iw_translator;


const char* StrategyParser::TAG_NAME  = "name";
const char* StrategyParser::TAG_CLASS = "class";
const char* StrategyParser::TAG_UTIL  = "utility";
const char* StrategyParser::TAG_COSTS = "costs";
const char* StrategyParser::TAG_DEPS  = "dependencies";
const char* StrategyParser::TAG_SRC   = "source";
const char* StrategyParser::TAG_BUP   = "bringup";
const char* StrategyParser::TAG_BDN   = "bringdown";

template <class T>
bool StrategyParser::parseNumber(XmlRpc::XmlRpcValue& node, T& v)
{
    if (node.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        v = static_cast<double>(node);
        return true;
    } else if (node.getType() == XmlRpc::XmlRpcValue::TypeInt) {
        v = static_cast<int>(node);
        return true;
    } else {
        return false;
    }
}

template <class T>
bool StrategyParser::parseNumber(
    XmlRpc::XmlRpcValue& node,
    const char* tag,
    T& out)
{
    if (!node.hasMember(tag)) {
        ROS_WARN("Strategy has no member '%s'.", tag);
        return false;
    }
    
    return parseNumber(node[tag], out);
}

bool StrategyParser::parseString(
    XmlRpc::XmlRpcValue& node,
    const char* tag,
    std::string& out)
{
    if (!node.hasMember(tag)) {
        ROS_WARN("Strategy has no member '%s'.", tag);
        return false;
    }

    XmlRpc::XmlRpcValue& v = node[tag];
    if (v.getType() != XmlRpc::XmlRpcValue::TypeString) {
        ROS_WARN("Strategy '%s' element is not a string.", tag);
        return false;
    }

    out = std::string(v);
    return true;
}

void StrategyParser::parseCosts(
    XmlRpc::XmlRpcValue& costs,
    std::vector<hbba_msgs::ResourceUsage>& out)
{
    if (costs.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_WARN("Resource definition is not a struct.");
        return;
    }
    //
    // Actually behaves as map<string, XmlRpcValue>:
    typedef XmlRpc::XmlRpcValue::iterator It;
    for (It i = costs.begin(); i != costs.end(); ++i) {
        const std::string&   key   = i->first;
        XmlRpc::XmlRpcValue& value = i->second;
        double v;
        if (!parseNumber(value, v)) {
            ROS_WARN(
                "Cost '%s' is not a numerical value.", 
                key.c_str());
        } else {
            int j = out.size();
            out.resize(j+1);
            hbba_msgs::ResourceUsage& cost = out[j];
            cost.id    = key;
            cost.value = v;
        }
    }
}

bool StrategyParser::parseSingle(
    XmlRpc::XmlRpcValue& node, 
    hbba_msgs::Strategy& strat)
{
    // Validation tests for the mandatory fields, then copy everything in the 
    // structure:
    
    if (node.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_ERROR("Strategy cannot be parsed: given object is not a struct.");
        return false;
    }

    std::string name;
    std::string cls;
    double      util;
    if (!parseString(node, TAG_NAME, name)) {
        ROS_ERROR("Could not parse strategy name.");
        return false;
    }
    if (!parseString(node, TAG_CLASS, cls)) {
        ROS_ERROR("Could not parse strategy class.");
        return false;
    }
    if (!parseNumber(node, TAG_UTIL, util)) {
        ROS_ERROR("Could not parse strategy utility.");
        return false;
    }

    strat.id            = std::string(name);
    strat.utility.id    = std::string(cls);
    strat.utility.value = util; 

    // Still perform validity tests, but only warn on non-mandatory fields:
    
    if (node.hasMember(TAG_COSTS)) {
        parseCosts(node[TAG_COSTS], strat.cost);
    }

    if (node.hasMember(TAG_DEPS)) {
        parseCosts(node[TAG_DEPS], strat.utility_min);
    }

    parseString(node, TAG_SRC, strat.source);
    parseString(node, TAG_BUP, strat.bringup_function);
    parseString(node, TAG_BDN, strat.bringdown_function);

    return true;
}

bool StrategyParser::parseArray(
    XmlRpc::XmlRpcValue& node,
    std::vector<hbba_msgs::Strategy>& strats)
{
    // Just call parseSingle for every member of the array, if it's one:
    
    if (node.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("Strategies cannot be parsed: given object is not an array.");
        return false;
    }

    for (int i = 0; i < node.size(); ++i) {
        hbba_msgs::Strategy strat;
        if (parseSingle(node[i], strat)) {
            strats.push_back(strat);
        }
    }

    return true;
}

