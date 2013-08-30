#include "emotion_generator.h"

EmotionGenerator::EmotionGenerator(ros::NodeHandle * n, std::string nodeName)
{
	timer_ = n->createTimer(ros::Duration(1),
			&EmotionGenerator::timerCB, this);
	n_ = n;
	nodeName_ = nodeName;

	pubEmotion = n_->advertise<hbba_msgs::EmotionIntensities>("emotions",100,true);
}

void EmotionGenerator::eventsCallback(const hbba_msgs::Event& msg)
{
	ROS_INFO("emotionGenerator event %i %s",msg.type, msg.desire_type.c_str());

	switch(msg.type)
	{
	case hbba_msgs::Event::EXP_ON:
	{
		exploitedDesires[msg.desire_type] = true;
		break;
	}
	case hbba_msgs::Event::EXP_OFF:
	{
		exploitedDesires[msg.desire_type] = false;
		break;
	}
	case hbba_msgs::Event::DES_ON:
	{
		activeDesires[msg.desire_type] = true;
		break;
	}
	case hbba_msgs::Event::DES_OFF:
	{
		activeDesires[msg.desire_type] = false;
		break;
	}
	}

	if(emotionMatrix.count(msg.desire_type) == 0)
	{
		std::string paramDesire = n_->getNamespace() + nodeName_ + "/" + msg.desire_type;
		//ROS_INFO("emotionMatrix %s",paramDesire.c_str());

		XmlRpc::XmlRpcValue emotionParam;
		n_->getParam(paramDesire,emotionParam);

		std::map<std::string,double> mapEmotion;

		for (std::map<std::string, XmlRpc::XmlRpcValue>::iterator it = emotionParam.begin(); it != emotionParam.end(); it++)
		{
			std::string emotion = paramDesire + "/" + (*it).first;
			double value = 0;
			n_->getParam(emotion,value);
			ROS_INFO("%s, %5.2f",(*it).first.c_str(), value);

			mapEmotion[(*it).first] = value;

			if(emotionIntensities.count((*it).first) == 0)
			{
				//initialize this emotion
				emotionIntensities[(*it).first] = 0;
			}
		}

		emotionMatrix[msg.desire_type] = mapEmotion;
	}

}

void EmotionGenerator::timerCB(const ros::TimerEvent&)
{

	generateEmotions();

	hbba_msgs::EmotionIntensities emotions;
	emotions_msgs::EmoIntensity intensity;
	//publish emotionIntensities
	for(std::map<std::string,double>::iterator it = emotionIntensities.begin() ; it != emotionIntensities.end() ; it++)
	{
		intensity.name = (*it).first;
		intensity.value = (*it).second;
		emotions.emotion.push_back(intensity);
	}

	pubEmotion.publish(emotions);
}

void EmotionGenerator::generateEmotions()
{
	//verify each active desires if they are exploited
	for(std::map<std::string,bool>::iterator it = activeDesires.begin() ; it != activeDesires.end() ; it++)
	{
		std::string desires = (*it).first;
		if((*it).second && exploitedDesires[desires])
		{
			//emotion are influence according to a matrix defined in yaml file
			for(std::map<std::string,double>::iterator it = emotionMatrix[desires].begin() ; it!= emotionMatrix[desires].end() ; it++)
			{
				std::string emotion = (*it).first;
				double emotionModulation = (*it).second;
				double emotionIntensity = emotionIntensities[emotion];
				emotionIntensity += emotionModulation;
			}
		}
	}
}
