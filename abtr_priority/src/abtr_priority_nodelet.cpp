#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <abtr_priority/generic.hpp>
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>

namespace abtr_priority
{
	/// \brief A generic, priority-based synchronized arbitration nodelet.
	///
    /// See the 'Generic' class for details.
	class GenericNodelet: public nodelet::Nodelet
	{
	public:
		virtual void onInit()
		{
            // NOTE: The root namespace will be the same as the nodelet manager.
			ros::NodeHandle n = getNodeHandle();
			ros::NodeHandle np = getPrivateNodeHandle();

            impl_.reset(new Generic(n, np));
		}
    private:
        boost::scoped_ptr<abtr_priority::Generic> impl_;
	};

}

PLUGINLIB_EXPORT_CLASS(abtr_priority::GenericNodelet, nodelet::Nodelet)

