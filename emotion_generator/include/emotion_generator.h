#include <ros/ros.h>
#include <ros/package.h>
#include <hbba_msgs/Event.h>
#include <hbba_msgs/EmotionIntensities.h>
#include <emotions_msgs/Intensity.h>
#include <std_msgs/Bool.h>

class EmotionGenerator
{
public:
	EmotionGenerator(ros::NodeHandle*,std::string);
	virtual ~EmotionGenerator(){;}

	void eventsCallback(const hbba_msgs::Event& msg);
	void getEmotionDesireRelation(std::string paramServerDesire);
	void timerCB(const ros::TimerEvent&);
	void emotionDecayCB(const ros::TimerEvent&);
	void generateEmotions();
	void emotionDecay();

private:

	std::map<std::string,bool> exploitedDesires;
	std::map<std::string,bool> activeDesires;

	std::map<std::string, std::map<std::string,double> >  emotionMatrix;

	std::map<std::string,double> emotionIntensities;

	ros::Timer timer_;
	ros::Timer timerDecay_;
	ros::Publisher pubEmotion;
	ros::Publisher pubEmotionFaceExpression;
	ros::Publisher pubDebugJoy;
	ros::Publisher pubDebugAnger;

	ros::Publisher pubDebugGotoExp;
	ros::Publisher pubDebugGotoDesire;

	ros::NodeHandle* n_;

	double emotionDecay_;
	bool debugEmotionGenerator_;
	std::string nodeName_;

};
