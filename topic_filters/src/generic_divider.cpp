#include "generic_divider.hpp"
#include <algorithm>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "generic_divider");

    std::vector<std::string> args(argc - 1);
    std::copy(&argv[1], &argv[argc], args.begin());

    ros::NodeHandle n, np("~");
    topic_filters::GenericDividerNode node(n, np, args);

    ros::spin();
    return 0;
}

