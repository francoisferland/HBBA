#ifndef SHAPESHIFTER_PUB_HPP
#define SHAPESHIFTER_PUB_HPP

#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <abtr_priority/backend_common.hpp>

namespace abtr_priority
{
	namespace impl
	{
		/// \brief Specialized advertizer for ShapeShifter.
		template <>
		ros::Publisher advertise<topic_tools::ShapeShifter>(ros::NodeHandle n,
			const std::string& topic, const topic_tools::ShapeShifter& msg);
	}
}

#endif

