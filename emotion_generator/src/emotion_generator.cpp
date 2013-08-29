#include "emotion_generator.h"

EmotionGenerator::EmotionGenerator(ros::NodeHandle * n)
{
}

void EmotionGenerator::eventsCallback(const hbba_msgs::Event& msg)
{
	ROS_INFO("emotionGenerator goto event %i",msg.type);

	if(msg.type == hbba_msgs::Event::EXP_ON )
	{
		//add to egosphere
		exploitedDesires["Goto"] = true;

	}
	else if(msg.type == hbba_msgs::Event::EXP_OFF )
	{
		exploitedDesires["Goto"] = false;
	}
}
