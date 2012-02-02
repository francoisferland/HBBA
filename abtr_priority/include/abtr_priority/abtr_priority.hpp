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
				ROS_INFO("Registering behavior %s", i->c_str());
				int p = getPriority(*i);
				if (p >= 0)
                {
					subscribers_.push_back(BehaviorSubPtr(
						new BehaviorSub(*this, n_, *i, p)));
                    ROS_INFO("Registration for %s done.", i->c_str());
                }
				else
					ROS_WARN("Invalid priority for topic %s", i->c_str());
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
			std::string paramname = pubname + "/abtr_priority";
			//ROS_INFO("Looking for priority in %s", paramname.c_str());
			n_.param(paramname, p, -1);
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

	namespace impl_
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

	/// \brief Priority-based arbitration back end template.
	///
	/// See GenericNodelet for an example of usage and description.
	template <class M>
	class BackEnd
	{
	public:
		typedef BackEnd<M> ThisType;
		typedef boost::shared_ptr<ThisType> Ptr;

		/// \brief Default constructor. Results in an unusable back end.
		BackEnd()
		{
            // Stop the output thread.
            running_ = false;
            thread_.join();
		}

		/// \brief Complete constructor.
		///
		/// \param n The ROS node handle to register topics with.
		/// \param topic_name The name of the command output topic.
		BackEnd(ros::NodeHandle& n, const std::string& topic_name = "abtr_cmd"): 
			n_(n), topic_name_(topic_name), last_cmd_(new M()), last_cmd_p_(-1)
		{
			double p;
			n_.param("abtr_period", p, 0.1);
            period_ = ros::Duration(p);
			n_.param("abtr_cycle_flush", cycle_flush_, false);
			timeout_ = ros::Duration(2.0 * p);

			//timer_ = n_.createTimer(ros::Duration(p), 
			//	&ThisType::timerCB, this);
			
			advertised_ = false;
            running_ = true;
            thread_ = boost::thread(boost::bind(&ThisType::threadLoop, this)); 

		}

		void addCommand(int priority, ros::Time time, 
			const typename M::ConstPtr& msg)
		{	
            mutex_t::scoped_lock lock(mutex_);
			// Make a copy of the pointer to the incoming command.
			// We need to keep the pointer since the actual data buffers are
			// kept by pointers and will be lost when we release the shared
			// pointer.
			// Some message types (e.g. ShapeShifter) don't have valid copy 
			// constructors.
			cmd_map_[priority] = cmd_pair_t(time, msg);
		}

		/// \brief Return a pointer to the current top priority command.
        ///
        /// If the list is empty, you will receive a default constructor-
        /// initialized command and p will be -1.
		const typename M::ConstPtr top(int& p) 
		{
            mutex_t::scoped_lock lock(mutex_);

            if (cmd_map_.size() == 0)
            {
                // Return the default command when the list is empty,
                // p will be -1.
                const typename M::ConstPtr def(new M());
                p = -1;
                return def;
            }

			// Return whatever is left at the top of the list.
			const typename M::ConstPtr cmd =
				cmd_map_.rbegin()->second.second;
			p = cmd_map_.rbegin()->first;
			return cmd;
		}

        /// \brief Return the last exploited command.
        const typename M::ConstPtr last(int* p = 0) 
        {
            mutex_t::scoped_lock lock(mutex_);

            if (p != 0)
                *p = last_cmd_p_;

            return last_cmd_;
        }

	private:
		void threadLoop()
		{
            while (running_)
            {
                period_.sleep();

                {
                    mutex_t::scoped_lock lock(mutex_);

                    if (!cycle_flush_)
                        clearOldCommands(ros::Time::now());

                    if (cmd_map_.empty())
                        continue;

                    // Return whatever is left at the top of the list.
                    std_msgs::Int32 p;
                    const typename M::ConstPtr cmd = top(p.data);

                    if (!advertised_)
                    {
                        pub_ = impl_::advertise<M>(n_, topic_name_, *cmd);
                        pub_priority_ = 
                            n_.advertise<std_msgs::Int32>("priority", 10);
                        advertised_ = true;
                    }
                    pub_.publish(cmd);
                    pub_priority_.publish(p);

                    last_cmd_ = cmd;
                    last_cmd_p_ = p.data;

                    if (cycle_flush_)
                        cmd_map_.clear();
                }
            }
		}

		/// \brief Look for expired commands in the map.
		///
		/// \brief now Current time to compare commands to.
		void clearOldCommands(const ros::Time& now)
		{
			std::vector<int> del_list;
			typename cmd_map_t::const_iterator i;
			for (i = cmd_map_.begin(); i != cmd_map_.end(); ++i)
				if ((now - i->second.first) > timeout_)
					del_list.push_back(i->first);

			std::vector<int>::const_iterator j;
			for (j = del_list.begin(); j != del_list.end(); ++j)
				cmd_map_.erase(*j);
		}

		ros::NodeHandle n_;
		ros::Duration timeout_;
		ros::Publisher pub_;
		ros::Publisher pub_priority_;
		std::string topic_name_;
		bool advertised_;
		bool cycle_flush_;

		typedef std::pair<ros::Time, typename M::ConstPtr> cmd_pair_t;
		typedef std::map<int, cmd_pair_t> cmd_map_t;
		cmd_map_t cmd_map_;

        typename M::ConstPtr last_cmd_;
        int last_cmd_p_;

        boost::thread thread_;
        bool running_;
        ros::Duration period_;
        typedef boost::recursive_mutex mutex_t;
        mutex_t mutex_;

	};

}

