#include <hbba_validation/cardreader_localizer_openni.hpp>

using namespace hbba_validation;

CardreaderLocalizerOpenNI::CardreaderLocalizerOpenNI(      ros::NodeHandle& n,
                                                     const ros::NodeHandle& np):
    openni_tools::RectToPose<CardreaderLocalizerOpenNI>(n)
{
    int t;
    np.param("threshold", t, 64);
    localizer_.threshold(t);

    np.param("min_c",      min_c_,       6);
    np.param("max_c",      max_c_,      30);
    np.param("led_radius", led_radius_,  3);

}

void CardreaderLocalizerOpenNI::imageCB(const sensor_msgs::Image::ConstPtr& i,
                                        const sensor_msgs::Image::ConstPtr& d)
{
    int x, y, c = localizer_.process(i, x, y);
    if (c > min_c_ && c < max_c_ && valid_cb_) {
        geometry_msgs::PoseStamped pose;
        if (getPoseStamped(cv::Rect(x - led_radius_, 
                                    y - led_radius_, 
                                    2 * led_radius_, 
                                    2 * led_radius_),
                           pose)) {
            valid_cb_(pose);
        } else {
            ROS_ERROR("Could not find pose from OpenNI image.");
        }

    } else if (invalid_cb_) {
        invalid_cb_();
    }
}

