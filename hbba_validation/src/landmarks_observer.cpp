#include <hbba_validation/landmarks_observer.hpp>
#include <irl1_interaction/common.hpp>

using namespace hbba_validation;

LandmarksObserver::LandmarksObserver(ros::NodeHandle& n, ros::NodeHandle& np)
{
    np.param("fixed_frame", fixed_frame_, std::string("/map"));
    np.param("robot_frame", robot_frame_, std::string("/base_link"));

    sub_qrcodes_ = n.subscribe("qrcodes_decoded", 
                               10,
                               &LandmarksObserver::codesCB, 
                               this);
}

void LandmarksObserver::codesCB(const std_msgs::String& msg)
{
    const std::string& code = msg.data;

    // Find out the latest robot's pose, save that as the landmark's
    // location.
    geometry_msgs::PoseStamped& p = map_[code];
    tf::StampedTransform        st;
    if (!irl1_interaction::latestTransform(tf_, 
                                           fixed_frame_, 
                                           robot_frame_, 
                                           p)) {
        // NOTE: We always save a pose, the zeroed timestamp serves as marking
        // a landmark with an unknown robot location.
        p.header.stamp = ros::Time(); // Zeroed timestamp.
    }
}

