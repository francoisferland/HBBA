#include <ros/ros.h>
#include "emotion_generator.h"

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "EmotionGenerator");

	ros::NodeHandle n;

	EmotionGenerator emotionGenerator(&n);

	ros::Subscriber subEvent = n.subscribe("events",100,&EmotionGenerator::eventsCallback,&emotionGenerator);

	ros::spin();

	return 0;
}
