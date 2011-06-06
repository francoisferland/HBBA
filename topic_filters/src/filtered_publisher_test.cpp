#include "topic_filters/filtered_publisher.hpp"
#include <std_msgs/String.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "filtered_publisher_test");
	ros::NodeHandle n;

	topic_filters::filtered_publisher pub1 = 
		n.advertise<std_msgs::String>("test_topic", 100);

	topic_filters::filtered_publisher
		pub2(n.advertise<std_msgs::String>("test_topic", 100), "~");

	std_msgs::String msg1, msg2;
	msg1.data = "Test1";
	msg2.data = "Test2";

	ros::Rate rate(10);

	while (ros::ok())
	{
		ros::spinOnce();
		pub1.publish(msg1);
		pub2.publish(msg2);
		rate.sleep();
	}
}

