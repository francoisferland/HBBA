#ifndef ASYNC_BACKEND_HPP
#define ASYNC_BACKEND_HPP

#include <ros/ros.h>
#include <map>
#include <tr1/unordered_map>
#include <std_msgs/Int32.h>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/thread.hpp>
#include "backend_common.hpp"

namespace abtr_priority
{
	/// \brief Priority-based asynchronous arbitration back end template.
	///
    /// Asynchronous arbitration means a command will be republished as soon as
    /// it's received by the front end(s), but identical commands will not be 
    /// repeated.
	/// See GenericNodelet for an example of usage and description.
	template <class M>
	class AsyncBackEnd
	{
	public:
		typedef AsyncBackEnd<M> ThisType;
		typedef boost::shared_ptr<ThisType> Ptr;

		/// \brief Default constructor. Results in an unusable back end.
		AsyncBackEnd()
		{
            // Stop the output thread.
            running_ = false;
            thread_.join();
		}

		/// \brief Complete constructor.
		///
		/// \param n The ROS node handle to register topics with.
        /// \param np Private node handle for parameters.
		/// \param topic_name The name of the command output topic.
		AsyncBackEnd(const ros::NodeHandle& n, 
            const ros::NodeHandle& np = ros::NodeHandle("~"), 
            const std::string& topic_name = "abtr_cmd"): 
			n_(n), topic_name_(topic_name), last_cmd_(new M()), last_cmd_p_(-1)
		{
			double p, ap;
			np.param("timeout", p, 1.0);
			timeout_ = ros::Duration(p);
            np.param("abtr_period", ap, 0.1);
            period_ = ros::Duration(ap);

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

            publishCommand();
		}

		/// \brief Return a pointer to the current top priority command.
        ///
        /// If the list is empty, you will receive a default constructor-
        /// initialized command and p will be -1.
        /// \param p A reference to a int to copy priority value into.
        /// \param time A pointer to a time structure to copy the reception
        /// time into.
		const typename M::ConstPtr top(int& p, ros::Time* time = 0) 
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
            typename cmd_map_t::const_reverse_iterator top = cmd_map_.rbegin();
			const typename M::ConstPtr cmd =
				top->second.second;
			p = top->first;
            if (time != 0)
                *time = top->second.first;
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
                    clearOldCommands(ros::Time::now());
                }
            }
		}

        /// \brief Publishes the highest-priority command.
        ///
        /// Filtering has to be done first.
        /// Only publishes a new message if the current top command is
        /// different from the last one.
        /// Saves the last command (last_cmd_, last_cmd_p_, last_cmd_time_) 
        /// at the end.
        void publishCommand()
        {
            mutex_t::scoped_lock lock(mutex_);

            // Return whatever is left at the top of the list.
            std_msgs::Int32 p;
            ros::Time cmd_time;
            const typename M::ConstPtr cmd = top(p.data, &cmd_time);

            if (!advertised_)
            {
                pub_ = impl::advertise<M>(n_, topic_name_, *cmd);
                pub_priority_ = 
                    n_.advertise<std_msgs::Int32>("priority", 10);
                advertised_ = true;
            }
            if (cmd_time != last_cmd_time_)
            {
                pub_.publish(cmd);
                pub_priority_.publish(p);
                last_cmd_ = cmd;
                last_cmd_p_ = p.data;
                last_cmd_time_ = cmd_time;
            }


        }

		/// \brief Look for expired commands in the map.
		///
        /// NOTE: Does not delete commands with priority <= 0
		/// \brief now Current time to compare commands to.
		void clearOldCommands(const ros::Time& now)
		{
            mutex_t::scoped_lock lock(mutex_);

			std::vector<int> del_list;
			typename cmd_map_t::const_iterator i;
			for (i = cmd_map_.begin(); i != cmd_map_.end(); ++i)
				if ((now - i->second.first) > timeout_ && (i->first > 0))
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

		typedef std::pair<ros::Time, typename M::ConstPtr> cmd_pair_t;
		typedef std::map<int, cmd_pair_t> cmd_map_t;
		cmd_map_t cmd_map_;

        typename M::ConstPtr last_cmd_;
        int last_cmd_p_;
        ros::Time last_cmd_time_;

        boost::thread thread_;
        bool running_;
        ros::Duration period_;
        typedef boost::recursive_mutex mutex_t;
        mutex_t mutex_;

	};

}

#endif

