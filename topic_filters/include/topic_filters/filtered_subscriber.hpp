#ifndef FILTERED_SUBSCRIBER_HPP
#define FILTERED_SUBSCRIBER_HPP

#include "topic_filters/SetState.h"
#include "topic_filters/SetDividerRate.h"
#include <ros/ros.h>

namespace topic_filters
{
	template <class T, class M> class filtered_subscriber_impl;

	template <class T, class M>
	class filtered_subscriber
	{
	public:
		filtered_subscriber(const std::string& topic, unsigned int queue_size,
			void (T::*fp)(M), 
			T* obj,
			const ros::TransportHints& th = ros::TransportHints()): 
			fp_(fp),
			obj_(obj),
			impl_(this, topic, queue_size, th)
		{
		}

		const filtered_subscriber<T,M> operator ()(M m)
		{
			(obj_->*fp_)(m);
			return *this;
		}

		void sub_cb(M m)
		{
			impl_.sub_cb(m);
		}

	private:
		void (T::*fp_)(M);
		T* obj_;

		filtered_subscriber_impl<T,M> impl_;	

	};

	template<class M>
	class filtered_subscriber<void, M> 
	{
	public:
		filtered_subscriber(const std::string& topic, unsigned int queue_size,
			void (*fp)(M), 
			const ros::TransportHints& th = ros::TransportHints()): 
			fp_(fp),
			impl_(this, topic, queue_size, th)
		{
		}

		const filtered_subscriber<void, M> operator ()(M m)
		{
			fp_(m);
			return *this;
		}

		void sub_cb(M m)
		{
			impl_.sub_cb(m);
		}

	private:
		void (*fp_)(M);

		filtered_subscriber_impl<void, M> impl_;	

	};

	template<class T, class M>
	class filtered_subscriber_impl
	{
	public:
		filtered_subscriber_impl(
			filtered_subscriber<T, M>* fs,
			const std::string& topic, 
			unsigned int queue_size,
			const ros::TransportHints& th):
			fs_(fs), 
			active_(true), rate_(1), count_(0)
		{
			ros::NodeHandle n;

			subscriber_ = n.subscribe(topic, queue_size,
				&filtered_subscriber_impl::sub_cb, this, th);

			std::string ns = "~" + topic;
			n = ros::NodeHandle(ns);
			
			srv_state_ = n.advertiseService("switch_set_state", 
				&filtered_subscriber_impl<T, M>::srv_state_cb, this);
			srv_rate_ = n.advertiseService("set_divider_rate", 
				&filtered_subscriber_impl<T, M>::srv_rate_cb, this);

			n.param("active", active_, true);
			n.param("divider_rate", rate_, 1);

		}
		~filtered_subscriber_impl()
		{
		}

		void sub_cb(M msg) 
		{
			if (active_ && (count_ == 0))
			{
				(*fs_)(msg);
			}

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

		ros::Subscriber subscriber_;
		ros::ServiceServer srv_state_;
		ros::ServiceServer srv_rate_;

		filtered_subscriber<T, M>* fs_;

		bool active_;
		int rate_;
		int count_;

	};
	
	template <class T, class M>
	struct fs_registry 
	{
		static void store(filtered_subscriber<T, M>* fs)
		{
			fs_registry<T,M>::fs_.list_.push_back(fs);
		}

	private:
		typedef std::vector< filtered_subscriber<T, M>* > list_t;

		~fs_registry()
		{
			typename list_t::iterator i;
			for (i = list_.begin(); i != list_.end(); ++i)
				delete *i;
		}

		list_t list_;

		static fs_registry<T, M> fs_;
	};
	template <class T, class M> fs_registry<T, M> fs_registry<T, M>::fs_;

	/// \brief Registers a filtered subscriber.
	///
	/// Only one filtered topic subscription per node.
	template <class T, class M>
	void register_fs(const std::string& topic, unsigned int ql,
		void (T::*fp)(M), T* obj) 
	{
		typedef filtered_subscriber<T, M> fs_t;
		
		fs_t* fs = new fs_t(topic, ql, fp, obj);
		fs_registry<T, M>::store(fs);

	}

	/// \brief Registers a filtered subscriber without an object reference.
	///
	/// Only one filtered topic subscription per node.
	template <class M>
	void register_fs(const std::string& topic, unsigned int ql,
		void (*fp)(M)) 
	{
		typedef filtered_subscriber<void, M> fs_t;
		
		fs_t* fs = new fs_t(topic, ql, fp);
		fs_registry<void, M>::store(fs);

	}

}

#endif

