#include <ros/ros.h>
#include <hbba_msgs/Event.h>
#include <hbba_msgs/EmotionIntensities.h>

class EmotionGenerator
{
public:
	EmotionGenerator(ros::NodeHandle*,std::string);
	virtual ~EmotionGenerator(){;}

	void eventsCallback(const hbba_msgs::Event& msg);
	void timerCB(const ros::TimerEvent&);
	void generateEmotions();

private:

	std::map<std::string,bool> exploitedDesires;
	std::map<std::string,bool> activeDesires;

	std::map<std::string, std::map<std::string,double> >  emotionMatrix;

	std::map<std::string,double> emotionIntensities;

	ros::Timer timer_;
	ros::Publisher pubEmotion;
	ros::NodeHandle* n_;

	std::string nodeName_;

};
