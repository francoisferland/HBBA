#include <abtr_priority/generic.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "abtr_priority_generic_node");

    ros::NodeHandle n, np("~");
    abtr_priority::Generic node(n, np);

    ros::spin();

    return 0;
}

