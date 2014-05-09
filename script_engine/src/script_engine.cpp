#include <script_engine/engine_v8.hpp>
#include <ros/ros.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "script_engine");

	script_engine::engine_v8 engine;

	ros::spin();

	return 0;
}
