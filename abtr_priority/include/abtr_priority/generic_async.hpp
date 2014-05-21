#ifndef GENERIC_ASYNC_HPP
#define GENERIC_ASYNC_HPP

#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include "frontend.hpp"
#include "async_backend.hpp"
#include "shapeshifter_pub.hpp"

namespace abtr_priority
{
	/// \brief A generic, priority-based asynchronous arbitration node.
	///
    /// This generic node can be used to arbitrate command feeds with
    /// different update periods.
    /// See the default, synchronized version for details on front end usage.
	///
	/// The node publishes a command with the highest priority value on the 
    /// "abtr_cmd" topic as soon as it is received by the front end module(s). 
    /// A command becomes invalid when its age is beyond the given time period.
    /// This is the "period" parameter (default: 1.00 s).
    /// This validity is verified periodically, at every "abtr_period" seconds
    /// (default: 0.1 s).
	/// Publication starts with the first valid command, and completely stops if
	/// every command has been deactivated.
	/// A good practice is to initialize the arbitration node with a default,
	/// safe command at priority 0.
    /// A command at priority 0 has infinite lifetime.
	///
	/// This class is an union of both the Frontend and AsyncBackend classes in 
    /// their default configuration with ShapeShifters for topic types.
	class GenericAsync
	{
	public:
        /// \brief Constructor.
        ///
        /// \param n  Node handle passed for topics.
        /// \param np Node handle passed for parameters.
	    GenericAsync(ros::NodeHandle& n, ros::NodeHandle& np)
		{
			back_.reset(new BackEndType(n, np, "abtr_cmd"));
			// WARNING! Can't remap subscription topic names in a nodelet ?
			// Works for the back end, doesn't for the front end.
			front_.reset(new FrontEndType(n, np, "cmd", 
				&BackEndType::addCommand, back_.get()));
		}

	private:
		typedef ASyncBackEnd<topic_tools::ShapeShifter> ABackEndType;
		BackEndType::Ptr back_;
		typedef FrontEnd<topic_tools::ShapeShifter, BackEndType> FrontEndType;
		FrontEndType::Ptr front_;
	};

}

#endif

