#include "shapeshifter_pub.hpp"

namespace abtr_priority
{
	namespace impl_
	{
        template <>
		ros::Publisher advertise<topic_tools::ShapeShifter>(ros::NodeHandle n,
			const std::string& topic, const topic_tools::ShapeShifter& msg)
		{
			return msg.advertise(n, topic, 10);
		}
	}
}

