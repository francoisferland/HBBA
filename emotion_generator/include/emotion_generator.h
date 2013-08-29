#include <ros/ros.h>
#include <hbba_msgs/Event.h>

class EmotionGenerator
{
public:
	EmotionGenerator(ros::NodeHandle*);
	virtual ~EmotionGenerator(){;}

	void eventsCallback(const hbba_msgs::Event& msg);

private:

	std::map<std::string,bool> exploitedDesires;

};
