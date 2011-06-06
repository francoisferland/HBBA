#include "topic_filters/filtered_subscriber.hpp"
#include <std_msgs/String.h>
#include <ros/ros.h>

struct A
{
	void cb(const std_msgs::String::ConstPtr& msg)
	{
		ROS_INFO("%s", msg->data.c_str());
	}
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "filtered_subscriber_test");
	ros::NodeHandle n;

	//topic_filters::filtered_subscriber<std_msgs::String> 
	//	sub("test_topic", 100, cb);
	A a;
	topic_filters::register_fs("test_topic", 100, &A::cb, &a);

	ros::spin();

}

