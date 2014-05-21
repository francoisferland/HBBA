#include <abtr_priority/generic_async.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "abtr_priority_generic_async_node");

    ros::NodeHandle n, np("~");
    abtr_priority GenericAsync node(n, np);

    ros::spin();

    return 0;
}

