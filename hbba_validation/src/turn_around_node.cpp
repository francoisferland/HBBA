#include <hbba_validation/turn_around.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turn_around_node");

    ros::NodeHandle n, np("~");
    hbba_validation::TurnAround node(n, np);

    ros::spin();

    return 0;
}

