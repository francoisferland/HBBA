
#include <ros/ros.h>
#include "hbba_msgs/Desire.h"
#include "hbba_msgs/ResourcesSet.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "core_node");
	ros::NodeHandle nh;

	ros::spin();
}
