#ifndef FILTERED_PUBLISHER_HPP
#define FILTERED_PUBLISHER_HPP

#include "topic_filters/SetState.h"
#include "topic_filters/SetDividerRate.h"
#include <ros/ros.h>

namespace topic_filters
{
	/// \brief A wrapper class to create filtered publishers with the same
	/// service interface as switch_filter.py.
	class filtered_publisher
	{
	public:
		filtered_publisher(): rate_(1), count_(0) 
		{
		}

		/// \brief Copy constructor
		///
		/// Enables direct initialization from a ros::Publisher. Won't actually
		/// be called as the compiler often optimize it away, but we still need
		// to provide it.
		filtered_publisher(const filtered_publisher& b)
		{
			publisher_ = b.publisher_;
			srv_state_ = b.srv_state_;
			srv_rate_  = b.srv_rate_;
			active_    = b.active_;
			rate_      = b.rate_;
		}
		
		filtered_publisher(const ros::Publisher& pub, const std::string& prefix
				= ""): 
			publisher_(pub), active_(true),
			rate_(1), count_(0)
		{
			ros::NodeHandle n(prefix + publisher_.getTopic());
			
			srv_state_ = n.advertiseService("switch_set_state", 
				&filtered_publisher::srv_state_cb, this);
			srv_rate_ = n.advertiseService("set_divider_rate", 
				&filtered_publisher::srv_rate_cb, this);
		
		}

		template<class T> 
		void publish(T& msg)
		{
			if (active_ && (count_ == 0))
				publisher_.publish(msg);

			count_ = ++count_ % rate_;
		}

		bool srv_state_cb(topic_filters::SetState::Request& req, 
			topic_filters::SetState::Response& res)
		{
			active_ = req.state;
			return true;
		}

		bool srv_rate_cb(topic_filters::SetDividerRate::Request& req, 
			topic_filters::SetDividerRate::Response& res)
		{
			rate_ = req.divisor;
			return true;
		}

	private:
		ros::Publisher publisher_;
		ros::ServiceServer srv_state_;
		ros::ServiceServer srv_rate_;

		bool active_;
		int rate_;
		int count_;

	};
}

#endif

