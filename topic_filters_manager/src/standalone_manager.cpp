#include "topic_filters_manager/manager.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "standalone_manager");

	topic_filters_manager::manager m;

	ros::spin();

}

