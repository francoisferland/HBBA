#ifndef GENERIC_HPP
#define GENERIC_HPP

#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <abtr_priority/frontend.hpp>
#include <abtr_priority/sync_backend.hpp>
#include <abtr_priority/shapeshifter_pub.hpp>

namespace abtr_priority
{
	/// \brief A generic, priority-based synchronized arbitration node.
	///
	/// The front end provides a service named "cmd/Register" to register a
	/// command source on the arbitration nodelet.
	/// The service accepts lists of topics.
	/// The front end looks for a parameter named "abtr_priority" under the
	/// registered topic's namespace, e.g. "/behavior_1/cmd_vel/abtr_priority".
	/// Registered topics without a priority attached are ignored and a warning 
	/// is issued.
	/// Note that priority values are cached and won't be refreshed until the
	/// arbitration nodelet is restarted.
	///
	/// The node periodically publishes the command with the highest priority 
	/// value on the "abtr_cmd" topic. 
	/// You can adjust the frequency with the abtr_period parameter of this 
	/// node. 
	/// The parameter, a double, represents the time period between 
	/// publications. 
	/// The default is 0.1, or 10 Hz.
	/// Publication starts with the first valid command, and completely stops if
	/// every command has been deactivated.
	/// A good practice is to initialize the arbitration node with a default,
	/// safe command at priority 0.
	///
	/// Commands are automatically deactivated when the node didn't receive a
	/// command with the same priority value for a period of twice the update
	/// cycle's period.
	///
	/// Priority values under 0 are considered invalid and will be automatically
	/// discarded.
	///
	/// The abtr_cycle_flush parameter indicates if the command queue should be
	/// flushed every cycle or not.
	/// This can be useful if you run an arbitration cycle slightly longer than
	/// incoming command streams and you don't want repeated commands from the
	/// arbitration output.
	/// In this mode, the automatic deactivation check is disabled.
	/// Deactivation entirely relies on the fact that a command will be flushed
	/// for the next cycle.
	/// The default value is false.
	///
	/// This class is an union of both the Frontend and SyncBackend classes in 
    /// their default configuration with ShapeShifters for topic types.
	class Generic
	{
	public:
        /// \brief Constructor.
        ///
        /// \param n  Node handle passed for topics.
        /// \param np Node handle passed for parameters.
	    Generic(ros::NodeHandle& n, ros::NodeHandle& np)
		{
			back_.reset(new BackEndType(n, np, "abtr_cmd"));
			// WARNING! Can't remap subscription topic names in a nodelet ?
			// Works for the back end, doesn't for the front end.
			front_.reset(new FrontEndType(n, np, "cmd", 
				&BackEndType::addCommand, back_.get()));
		}

	private:
		typedef SyncBackEnd<topic_tools::ShapeShifter> BackEndType;
		BackEndType::Ptr back_;
		typedef FrontEnd<topic_tools::ShapeShifter, BackEndType> FrontEndType;
		FrontEndType::Ptr front_;
	};

}

#endif

