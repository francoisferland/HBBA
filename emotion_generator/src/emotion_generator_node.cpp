#include <ros/ros.h>
#include "emotion_generator.h"

int main(int argc, char *argv[])
{

	std::string nodeName = "EmotionGenerator";
	ros::init(argc, argv, nodeName);

	ros::NodeHandle n;

	EmotionGenerator emotionGenerator(&n,nodeName);

	ros::Subscriber subEvent = n.subscribe("events",100,&EmotionGenerator::eventsCallback,&emotionGenerator);

	ros::spin();

	return 0;
}
