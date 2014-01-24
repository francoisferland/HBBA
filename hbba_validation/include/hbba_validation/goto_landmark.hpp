#ifndef GOTO_LANDMARK_HPP
#define GOTO_LANDMARK_HPP

#include "landmarks_observer.hpp"
#include <iw/events_filter.hpp>
#include <hbba_msgs/AddDesires.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/function.hpp>

namespace hbba_validation
{
    /// \brief A simple motivation module to reach observed landmarks.
    ///
    /// Can be part of a larger motivation module or used in a standalone
    /// fashion.
    /// Integrates a LandmarksObserver, produces navigation goals based on
    /// previously decoded landmarks.
    /// Invalid landmarks should be handled by the motivation module that
    /// integrates this one and thus normally interacts with someone.
    /// 
    /// Topics (see LandmarksObserver for others):
    ///  - landmark_goal: std_msgs/String, external interface to navigation goal
    ///                   production.
    ///  - events:        HBBA events, used to track reached goals.
    ///
    class GoToLandmark
    {
    private:
        ros::Subscriber    sub_goal_;
        ros::ServiceClient scl_add_;
        LandmarksObserver  obs_;
        iw::EventsFilter   events_filter_;

        boost::function<void (const std::string&)> cb_;
        std::string                                code_; // Current goal code.

        hbba_msgs::AddDesires           desires_req_;
        std::vector<hbba_msgs::Desire>& desires_set_;
        enum {
            DES_GOTO = 0
        };

    public:
        /// \brief Constructor.
        ///
        /// \param n  Node handle for topics.
        /// \param np Node handle for parameters.
        GoToLandmark(ros::NodeHandle& n, ros::NodeHandle& np);

        /// \brief Go to the specified landmark.
        ///
        /// If the landmark is known and has a valid pose, this will
        /// automatically produce a navigation goal and return true.
        /// Return false if the goal is invalid.
        bool goTo(const std::string& code);

        /// \brief Return a reference to the LandmarksObserver instance.
        LandmarksObserver&       observer()       { return obs_; } 
        /// \brief Return a const reference to the LandmarksObserver instance.
        const LandmarksObserver& observer() const { return obs_; } 

        /// \brief Register a callback for when the goal has been reached.
        ///
        /// Calls the callback with the reached landmark code.
        template <class T>
        void registerCB(void (T::*fun)(const std::string&), T* obj)
        {
            cb_ = boost::bind(fun, obj, _1);
        }

    private:
        void goalCB(const std_msgs::String& msg);
        void newLandmarkCB(const std::string& code);
        void eventCB(const hbba_msgs::Event& evt);

        void gotoDesire(const geometry_msgs::PoseStamped& goal);

    };
}

#endif

