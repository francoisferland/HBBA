#include <iw_translator/strategy_parser.hpp>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    // A simple ROS node to test the strategy parser.
    // Will test two parameters in its private node handle, if their available:
    //  "strategy", to test parseSimple(...).
    //  "strategies", to test parseArray(...).

    ros::init(argc, argv, "test_strategy_parser");

    ros::NodeHandle np("~");

    if (np.hasParam("strategy")) {
        hbba_msgs::Strategy strat;
        XmlRpc::XmlRpcValue node; 
        np.getParam("strategy", node);
        if (iw_translator::StrategyParser::parseSingle(node, strat)) {
            ROS_INFO("Parsing succeeded.");
        } else {
            ROS_ERROR("Parsing failed.");
        }
    }

    return 0;

}

