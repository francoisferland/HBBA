#include <hbba_validation/goto_landmark.hpp>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "goto_landmark");

    ros::NodeHandle n, np("~");
    hbba_validation::GoToLandmark node(n, np);

    ros::spin();

    return 0;
}

