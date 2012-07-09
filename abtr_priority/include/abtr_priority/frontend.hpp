#ifndef FRONTEND_HPP
#define FRONTEND_HPP

#include <ros/ros.h>
#include <map>
#include <tr1/unordered_map>
#include "abtr_priority/RegisterBehavior.h"
#include <std_msgs/Int32.h>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/thread.hpp>

namespace abtr_priority
{
	/// \brief Front end template for priority-based arbitration.
	///
	/// The front end manages topic subscription and priority mapping.
	/// See GenericNodelet for an example of usage.
	///
	/// M is the incoming command messages type, T the callback receiver
	/// type.
	template <class M, class T>
	class FrontEnd
	{
	public:
		typedef FrontEnd<M, T> ThisType;
		typedef boost::shared_ptr<ThisType> Ptr;
		typedef void (T::*CallbackType)(int, ros::Time, const
			boost::shared_ptr<const M>&);

		/// \brief Default constructor, results in an unsuable front end.
		FrontEnd()
		{
		}

		/// \brief Complete constructor.
		///
		/// \param n The ROS NodeHandle to subscribe from.
		/// \param topic_name The name of the commands topic for the service
		/// namespace.
		/// \param fp Callback to call with incoming (valid) commands. First
		/// parameter is the priority, the second the message time stamp and the
		/// third the actual message.
		/// \param fp_obj The instance of T to use the callback on.
		FrontEnd(ros::NodeHandle& n, 
			const std::string topic_name,
			void (T::*fp)(int, ros::Time, const boost::shared_ptr<const M>&),
			T* fp_obj): 
			n_(n),
			srv_(n_.advertiseService(topic_name + "/register", 
				&ThisType::registerCB, this)),
			//sub_(n_.subscribe(topic_name, 10, &ThisType::cmdCB, this)),
			fp_(fp), fp_obj_(fp_obj)
		{
		}

	private:
		bool registerCB(RegisterBehavior::Request& req,
				RegisterBehavior::Response&)
		{
			typedef std::vector<std::string>::const_iterator i_t;
			for (i_t i = req.topics.begin(); i != req.topics.end(); ++i)
			{
				int p = getPriority(*i);
				if (p >= 0)
                {
					subscribers_.push_back(BehaviorSubPtr(
						new BehaviorSub(*this, n_, *i, p)));
                    ROS_INFO("Registered topic %s.", i->c_str());
                }
				else
					ROS_ERROR("Invalid priority for topic %s", i->c_str());
			}

			return true;
		}

		void command(const typename M::ConstPtr& msg, int p)
		{
			(fp_obj_->*fp_)(p, ros::Time::now(), msg);
		}

		/// \brief Retrieves the priority from the publisher's name. Returns -1
		/// if the priority isn't found.
		int getPriority(const std::string& pubname) const
		{
			int p;
            ros::NodeHandle np(pubname);
			np.param("abtr_priority", p, -1);
			return p;
		}

		/// \brief Private struct to handle callback with priorities.
		struct BehaviorSub
		{
			BehaviorSub(ThisType& f, ros::NodeHandle& n, 
				const std::string t, int p): 
				sub_(n.subscribe(t, 10, &BehaviorSub::CB, this)),
				frontend_(f),
				priority_(p)
			{
			}

			void CB(const typename M::ConstPtr& msg)
			{
				frontend_.command(msg, priority_);
			}

		private:
			ros::Subscriber sub_;
			ThisType& frontend_;
			int priority_;

		};

		typedef boost::shared_ptr<BehaviorSub> BehaviorSubPtr;
		std::vector<BehaviorSubPtr> subscribers_;

		ros::NodeHandle n_;
		ros::ServiceServer srv_;

		typedef std::tr1::unordered_map<std::string, int> priority_map_t;
		priority_map_t priority_map_;

		CallbackType fp_;
		T* fp_obj_;

	};

}

#endif

