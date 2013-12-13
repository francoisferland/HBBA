#ifndef LANDMARKS_OBSERVER_HPP
#define LANDMARKS_OBSERVER_HPP

#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>

namespace hbba_validation
{
    /// \brief A simple landmarks observer that records decoded QR codes and
    /// saves the latest robot's location they were seen.
    ///
    /// Topics:
    ///  - qrcode_decoded: std_msgs/String, decoded QR codes.
    ///  - /tf:            Used by TransformListener.
    ///
    /// Parameters:
    ///  - fixed_frame: The global reference frame.
    ///                 Default: '/map', but requires SLAM.
    ///  - robot_frame: The robot's frame to use when locating the robot.
    ///                 Default: '/base_link'.
    ///
    class LandmarksObserver
    {
    public:
        typedef std::map<std::string, geometry_msgs::PoseStamped> MapType;

    private:
        ros::Subscriber       sub_qrcodes_;
        tf::TransformListener tf_;

        std::string fixed_frame_;
        std::string robot_frame_;

        MapType map_;
        
        boost::function<void (const std::string&)> cb_;

    public:
        /// \brief Constructor.
        ///
        /// \param n  Node handle for topics.
        /// \param np Node handle for parameters.
        LandmarksObserver(ros::NodeHandle& n, ros::NodeHandle& np);

        /// \brief Return a const reference to the map of known landmarks.
        ///
        /// Note: Poses contained in the map can be invalid if the robot's
        /// location could not be found. 
        /// This can be verified by looking at the pose's timestamp: A zero
        /// value indicates a null pose.
        const MapType& landmarks() const { return map_; }

        /// \brief Return true if a landmark with the given code is known.
        ///
        /// NOTE: Does not guarantee that the landmark's pose is valid.
        bool hasLandmark(const std::string& code) const {
            MapType::const_iterator i = map_.find(code);
            return (i != map_.end());
        }

        /// \brief Save the current location as a custom landmark.
        ///
        /// Warning: Might save an invalid pose (header.stamp == 0) if it could
        /// not be obtained.
        ///
        /// \param code      The landmark identifier.
        /// \param enable_cb Enable callback firing if it's a newly discovered
        ///                  code.
        void saveLandmark(const std::string& code, bool enable_cb = true);

        /// \brief Register a callback for newly-registered landmarks.
        ///
        /// The callback is called from the decoded codes callback, and the map
        /// obtained from landmarks() will be fully updated.
        template <class T>
        void registerCB(void (*fun)(const std::string&), T* obj)
        {
            cb_ = boost::bind(fun, obj, _1);
        }

    private:
        void codesCB(const std_msgs::String& msg);

    };
}

#endif

