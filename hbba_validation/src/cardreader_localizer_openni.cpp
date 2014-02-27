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
    ROS_DEBUG_THROTTLE(1.0,
                       "localizer openni (x, y, c): (%i, %i, %i)",
                       x,
                       y,
                       c);
    if (c > min_c_ && c < max_c_ && valid_cb_) {
        geometry_msgs::PoseStamped pose;
        if (getPoseStamped(cv::Rect(std::max(x - led_radius_, 0), 
                                    std::max(y - led_radius_, 0), 
                                    std::min(2 * led_radius_, int(d->width)), 
                                    std::min(2 * led_radius_, int(d->height))),
                           pose)) {
            valid_cb_(pose);
        } else {
            ROS_ERROR("Could not find pose from OpenNI image.");
        }

    } else if (invalid_cb_) {
        invalid_cb_();
    }
}

