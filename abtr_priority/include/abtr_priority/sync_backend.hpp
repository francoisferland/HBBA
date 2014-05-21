#ifndef SYNC_BACKEND_HPP
#define SYNC_BACKEND_HPP

#include <ros/ros.h>
#include <map>
#include <tr1/unordered_map>
#include <std_msgs/Int32.h>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/thread.hpp>
#include "backend_common.hpp"

namespace abtr_priority
{
	/// \brief Priority-based synchronized arbitration back end template.
	///
	/// See GenericNodelet for an example of usage and description.
	template <class M>
	class SyncBackEnd
	{
	public:
		typedef SyncBackEnd<M> ThisType;
		typedef boost::shared_ptr<ThisType> Ptr;

		/// \brief Default constructor. Results in an unusable back end.
		SyncBackEnd()
		{
            // Stop the output thread.
            running_ = false;
            thread_.join();
		}

		/// \brief Complete constructor.
		///
		/// \param n The ROS node handle to register topics with.
		/// \param np The ROS node handle for parameters.
		/// \param topic_name The name of the command output topic.
		SyncBackEnd(ros::NodeHandle& n, ros::NodeHandle& np, 
            const std::string& topic_name = "abtr_cmd"): 
                n_(n), topic_name_(topic_name), last_cmd_(new M()), last_cmd_p_(-1)
		{
			double p;
			np.param("abtr_period", p, 0.1);
            period_ = ros::Duration(p);
			np.param("abtr_cycle_flush", cycle_flush_, false);
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
                    {
                        ROS_DEBUG(
                            "Arbitration command map empty, "
                            "skipping publication.");
                        continue;
                    }

                    // Return whatever is left at the top of the list.
                    std_msgs::Int32 p;
                    const typename M::ConstPtr cmd = top(p.data);

                    if (!advertised_)
                    {
                        pub_ = impl::advertise<M>(n_, topic_name_, *cmd);
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

#endif

