#include <hbba_validation/cardreader_localizer_imv.hpp>

using namespace hbba_validation;

CardreaderLocalizerIMV::CardreaderLocalizerIMV(      ros::NodeHandle& n,
                                               const ros::NodeHandle& np,
                                               const Mode             mode):
    mode_(mode)
{
    double imv_pan, imv_tilt, imv_zoom;
    np.param("imv_pan",  imv_pan,   0.0);
    np.param("imv_tilt", imv_tilt,  0.0);
    np.param("imv_zoom", imv_zoom, 90.0);
    np.param("min_c",    min_c_,      6);
    np.param("max_c",    max_c_,     30);
    imv_proc_.mode(imv_camera::MODE_PTZ);
    imv_proc_.pan(imv_pan);
    imv_proc_.tilt(imv_tilt);
    imv_proc_.zoom(imv_zoom);

    sub_image_ = n.subscribe("image_imv_raw", 
                             1, 
                             &CardreaderLocalizerIMV::imageCB, 
                             this);

    ros::NodeHandle npp(np); // Done to avoid casting away constness.
    pub_proc_ = npp.advertise<sensor_msgs::Image>("img_proc", 1);
}

void CardreaderLocalizerIMV::imageCB(const sensor_msgs::Image& img)
{
    sensor_msgs::Image::Ptr img_proc(new sensor_msgs::Image());
    if (!imv_proc_.process(img, *img_proc)) {
        ROS_ERROR("Could not process IMV raw image, "
                  "will not run localizing algorithm.");
        return;
    }

    if (pub_proc_.getNumSubscribers()) {
        pub_proc_.publish(img_proc);
    }

    int r_x, r_y, r_c = 0;
    int g_x, g_y, g_c = 0;
    
    switch (mode_) {
        case MODE_BOTH:
            localizer_.processBoth(img_proc, r_x, r_y, r_c, g_x, g_y, g_c);
            break;
        case MODE_SINGLE_RED:
            r_c = localizer_.process(img_proc, r_x, r_y, false);
            break;
        case MODE_SINGLE_GREEN:
            g_c = localizer_.process(img_proc, g_x, g_y, true);
            break;
        default:
            break;
    };

    ROS_DEBUG_THROTTLE(1.0,
                       "imv detector (r_x, r_y, r_c) (min, max): "
                       "(%i, %i, %i) (%i, %i)",
                       r_x,
                       r_y,
                       r_c,
                       min_c_,
                       max_c_);
    ROS_DEBUG_THROTTLE(1.0,
                       "imv detector (g_x, g_y, g_c): (%i, %i, %i)",
                       g_x,
                       g_y,
                       g_c);

    if ((r_c > min_c_) && (r_c < max_c_)) {
        callCB(img_proc, r_x, r_y, false);
    }
    if ((g_c > min_c_) && (g_c < max_c_)) {
        callCB(img_proc, g_x, g_y, true);
    }
}

void CardreaderLocalizerIMV::callCB(sensor_msgs::Image::Ptr img_proc,
                                    int                     x, 
                                    int                     y, 
                                    bool                    green)
{
    if (!cb_) {
        return;
    }

    double p = 0.0;
    double t = 0.0;
    if (!imv_proc_.orientationFromOutputPosition(x, y, p, t)) {
        ROS_WARN("Could not convert (%i, %i) into proper pan/tilt angles.",
                 x, 
                 y);
    }

    cb_(*img_proc, p, t, green);

}
