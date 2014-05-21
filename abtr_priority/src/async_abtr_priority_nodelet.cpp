#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <abtr_priority/async_backend.hpp>
#include <abtr_priority/generic_async.hpp>
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>

namespace abtr_priority
{
	/// \brief A generic, priority-based asynchronous arbitration nodelet.
	///
    /// See the GenericAsync class for details.
	class GenericAsyncNodelet: public nodelet::Nodelet
	{
	public:
		virtual void onInit()
		{
            // NOTE: The root namespace will be the same as the nodelet manager.
			ros::NodeHandle n = getNodeHandle();
            ros::NodeHandle np = getPrivateNodeHandle();

            impl_.reset(new GenericAsync(n, np);
		}

	private:
        boost::scoped_ptr<GenericAsync> impl_;
	};

}

PLUGINLIB_DECLARE_CLASS(abtr_priority, GenericAsync,
	abtr_priority::GenericAsyncNodelet, nodelet::Nodelet)

