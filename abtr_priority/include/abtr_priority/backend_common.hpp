#ifndef BACKEND_COMMON_HPP
#define BACKEND_COMMON_HPP

#include <ros/ros.h>

namespace abtr_priority
{
	namespace impl
	{
		/// \brief Template function to create a publisher from a generic topic
		/// type. 
		///
		/// This is mainly provided for topic types like ShapeShifter who needs
		/// special care when advertising a topic.
		template <class M>
		ros::Publisher advertise(ros::NodeHandle n, const std::string& topic, 
			const M& = M())
		{
			return n.advertise<M>(topic, 10);
		}

	}

}

#endif

