#include <hbba_validation/cardreader_localizer_imv.hpp>

using namespace hbba_validation;

CardreaderLocalizerIMV::CardreaderLocalizerIMV(      ros::NodeHandle& n,
                                               const ros::NodeHandle& np,
                                               const bool             gd):
    green_(gd)
{
    double imv_pan, imv_tilt, imv_zoom;
    np.param("imv_pan",  imv_pan,   0.0);
    np.param("imv_tilt", imv_tilt,  0.0);
    np.param("imv_zoom", imv_zoom, 90.0);
    imv_proc_.mode(imv_camera::MODE_PTZ);
    imv_proc_.pan(imv_pan);
    imv_proc_.tilt(imv_tilt);
    imv_proc_.zoom(imv_zoom);

    sub_image_ = n.subscribe("image_imv_raw", 
                             1, 
                             &CardreaderLocalizerIMV::imageCB, 
                             this);
}

void CardreaderLocalizerIMV::imageCB(const sensor_msgs::Image& img)
{
    sensor_msgs::Image::Ptr img_proc(new sensor_msgs::Image());
    if (!imv_proc_.process(img, *img_proc)) {
        ROS_ERROR("Could not process IMV raw image, "
                  "will not run localizing algorithm.");
        return;
    }

    int x, y, c = localizer_.process(img_proc, x, y);

    if ((c > min_c_) && (c < max_c_)) {
        double p = 0.0;
        double t = 0.0;
        if (!imv_proc_.orientationFromOutputPosition(x, y, p, t)) {
            ROS_WARN("Could not convert (%i, %i) into proper pan/tilt angles.",
                     x, 
                     y);
        }

        if (cb_) {
            cb_(*img_proc, p, t);
        }
    }
}


