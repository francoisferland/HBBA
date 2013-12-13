#ifndef GOTO_LANDMARK_HPP
#define GOTO_LANDMARK_HPP

#include "landmarks_observer.hpp"
#include <hbba_msgs/AddDesires.h>
#include <geometry_msgs/PoseStamped.h>

namespace hbba_validation
{
    /// \brief A simple motivation module to reach observed landmarks.
    ///
    /// Can be part of a larger motivation module or used in a standalone
    /// fashion.
    /// Integrates a LandmarksObserver, produces navigation goals based on
    /// previously decoded landmarks.
    /// Also produces desires to tell when a new landmark has been discovered.
    /// Invalid landmarks should be handled by the motivation module that
    /// integrates this one and thus normally interacts with someone.
    /// 
    /// Topics (see LandmarksObserver for others):
    ///  - landmark_goal: std_msgs/String, external interface to navigation goal
    ///                   production.
    ///
    class GoToLandmark
    {
    private:
        ros::Subscriber    sub_goal_;
        ros::ServiceClient scl_add_;
        LandmarksObserver  obs_;

        hbba_msgs::AddDesires           desires_req_;
        std::vector<hbba_msgs::Desire>& desires_set_;
        enum {
            DES_GOTO = 0,
            DES_SAY  = 1
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

    private:
        void goalCB(const std_msgs::String& msg);
        void newLandmarkCB(const std::string& code);

        void sayDesire(const std::string& data);
        void gotoDesire(const geometry_msgs::PoseStamped& goal);

    };
}

#endif

