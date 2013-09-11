#include <iw_translator/strategy_parser.hpp>
#include <iw_translator/solver_model.hpp>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    // A simple ROS node to test the strategy parser and static model builder.
    // Will test two parameters in "solver_model" scope, if their available:
    //  "strategy", to test parseSimple(...).
    //  "strategies", to test parseArray(...).

    ros::init(argc, argv, "test_strategy_parser");

    ros::NodeHandle np("solver_model");

    if (np.hasParam("strategy")) {
        hbba_msgs::Strategy strat;
        XmlRpc::XmlRpcValue node; 
        np.getParam("strategy", node);
        if (iw_translator::StrategyParser::parseSingle(node, strat)) {
            ROS_INFO("Single strategy parsing succeeded.");
        } else {
            ROS_ERROR("Single strategy parsing failed.");
        }
    }

    if (np.hasParam("strategies")) {
        std::vector<hbba_msgs::Strategy> strats;
        XmlRpc::XmlRpcValue node; 
        np.getParam("strategies", node);
        if (iw_translator::StrategyParser::parseArray(node, strats)) {
            ROS_INFO(
                "Array parsing finished, %lu element[s] added, "
                "will try building model...",
                strats.size());
            std::vector<hbba_msgs::ResourceUsage> zero_caps;
            iw_translator::SolverModel model(strats, zero_caps);

            ROS_INFO("Cost matrix C:\n%s",         model.cAsCSV().c_str());
            ROS_INFO("Utility matrix U:\n%s",      model.uAsCSV().c_str());
            ROS_INFO("Requirements matrix R:\n%s", model.rAsCSV().c_str());
            ROS_INFO("Combined matrix UR:\n%s",    model.urAsCSV().c_str());

        } else {
            ROS_ERROR("Array parsing failed.");
        }
    }

    return 0;

}

