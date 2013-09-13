#include "emotion_generator.h"
#include <ros/package.h>

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
	//ROS_INFO("emotionGenerator event %i %s %s",msg.type, msg.desire.c_str(), msg.desire_type.c_str());

	switch(msg.type)
	{
	case hbba_msgs::Event::EXP_ON:
	{
		exploitedDesires[msg.desire_type] = true;
		if(msg.desire_type.compare("Wander") == 0)
		{
			ROS_INFO("exp on wander");
		}
		break;
	}
	case hbba_msgs::Event::EXP_OFF:
	{
		exploitedDesires[msg.desire_type] = false;
		if(msg.desire_type.compare("Wander") == 0)
		{
			ROS_INFO("exp off wander");
		}
		break;
	}
	case hbba_msgs::Event::DES_ON:
	{
		activeDesires[msg.desire_type] = true;
		if(msg.desire_type.compare("Wander") == 0)
		{
			ROS_INFO("des on wander");
		}
		break;
	}
	case hbba_msgs::Event::DES_OFF:
	{
		activeDesires[msg.desire_type] = false;
		if(msg.desire_type.compare("Wander") == 0)
		{
			ROS_INFO("des off wander");
		}
		break;
	}
	case hbba_msgs::Event::INT_ON:
	{
		if(msg.desire_type.compare("Wander") == 0)
		{
			ROS_INFO("int on wander");
		}
		break;
	}
	case hbba_msgs::Event::INT_OFF:
	{
		if(msg.desire_type.compare("Wander") == 0)
		{
			ROS_INFO("int off wander");
		}
		break;
	}
	}

	if(!msg.desire_type.empty() && emotionMatrix.count(msg.desire_type) == 0)
	{
		std::string paramDesire = n_->getNamespace() + nodeName_ + "/" + msg.desire_type;
		//ROS_INFO("emotionMatrix %s",paramDesire.c_str());

		XmlRpc::XmlRpcValue emotionParam;
		n_->getParam(paramDesire, emotionParam);

		std::map<std::string,double> mapEmotion;

		for (std::map<std::string, XmlRpc::XmlRpcValue>::iterator it = emotionParam.begin(); it != emotionParam.end(); it++)
		{
			double emotionFactor = 0;
			if((*it).second.getType() == XmlRpc::XmlRpcValue::TypeInt)
			{
				int value = static_cast<int>((*it).second);
				//ROS_INFO("factor %s %s %i",msg.desire_type.c_str(),(*it).first.c_str(), value);
				emotionFactor = (double)value;

			}
			else if ((*it).second.getType() == XmlRpc::XmlRpcValue::TypeDouble)
			{
				emotionFactor = static_cast<double>((*it).second);
				//ROS_INFO("factor %s %s %5.2f",msg.desire_type.c_str(),(*it).first.c_str(), emotionFactor);
			}

			mapEmotion[(*it).first] = emotionFactor;

			if(emotionIntensities.count((*it).first) == 0)
			{
				//initialize this emotion
				emotionIntensities[(*it).first] = 0;
			}

		}
		if(mapEmotion.size() > 0)
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
				if(emotionIntensity > 100)
					emotionIntensity = 100;
				else if(emotionIntensity < 0)
					emotionIntensity = 0;

				emotionIntensities[emotion] = emotionIntensity; //update in map
			}
		}
		//if there is a desire but its not exploited, the emotion are influence according to the opposite values in matrix
		else if( ((*it).second && exploitedDesires.count(desires) == 0) || ((*it).second && exploitedDesires[desires] == false) )
		{
			for(std::map<std::string,double>::iterator it = emotionMatrix[desires].begin() ; it!= emotionMatrix[desires].end() ; it++)
			{
				std::string emotion = (*it).first;
				double emotionModulation = (*it).second;
				double emotionIntensity = emotionIntensities[emotion];
				emotionIntensity -= emotionModulation;
				if(emotionIntensity > 100)
					emotionIntensity = 100;
				else if(emotionIntensity < 0)
					emotionIntensity = 0;

				emotionIntensities[emotion] = emotionIntensity; //update in map

			}
		}
	}
}

void emotionDecay()
{

}
