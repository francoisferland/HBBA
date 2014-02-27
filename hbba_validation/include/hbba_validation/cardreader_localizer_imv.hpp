#ifndef CARDREADER_LOCALIZER_IMV_HPP
#define CARDREADER_LOCALIZER_IMV_HPP

#include "cardreader_localizer.hpp"
#include <imv_camera/imv_camera_processor.hpp>
#include <ros/ros.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>

namespace hbba_validation
{
    /// \brief A CardreaderLocalizer wrapper with a built-in IMV processor.
    ///
    /// Topics:
    ///  - image_imv_raw: Raw (panamorph) image input.
    ///  - ~img_proc:     Processed (cropped) image output.
    ///
    /// Parameters:
    ///  - imv_pan:     Pan angle for IMV processor, in degrees.
    ///                 Default: 0.0.
    ///  - imv_tilt:    Tilt angle for IMV processor, in degrees.
    ///                 Default: 0.0.
    ///  - imv_zoom:    Zoom angle for IMV processor, in degrees.
    ///                 Default: 90.0.
    ///  - threshold:   Threshold value for pixel detection, see
    ///                 CardreaderLocalizer for details.
    ///                 Default: 64.
    ///  - min_c:       Minimum detection pixel count for the red LED.
    ///                 Default: 6.
    ///  - max_c:       Maximum detection pixel count for the red LED.
    ///                 Too many pixels indicates more than a single red light
    ///                 source.
    ///                 Default: 30.
    ///
    class CardreaderLocalizerIMV
    {
    public:
        enum Mode
        {
            MODE_SINGLE_RED,
            MODE_SINGLE_GREEN,
            MODE_BOTH
        };

    private:
        ros::Subscriber                sub_image_;
        ros::Publisher                 pub_proc_;
        imv_camera::IMVCameraProcessor imv_proc_;
        CardreaderLocalizer            localizer_;

        Mode mode_;
        int  min_c_;
        int  max_c_;

        boost::function<void (const sensor_msgs::Image&,
                                    double,
                                    double,
                                    bool)>               cb_;

    public:
        /// \brief Constructor.
        ///
        /// \param n    Node handle for topics.
        /// \param np   Node handle for parameters.
        /// \param mode Localization mode (see enum), default: MODE_SINGLE_RED.
        CardreaderLocalizerIMV(      ros::NodeHandle& n,
                               const ros::NodeHandle& np,
                               const Mode             mode = MODE_SINGLE_RED);

        /// \brief Register a callback on valid detections.
        ///
        /// Your callback will receive a const reference to the processed image,
        /// pan and tilt angles of the LED in the camera's reference frame, and
        /// a boolean indicating if it's a green (true) or red (false) one.
        template <class T>
        void registerCB(void (T::*fun)(const sensor_msgs::Image&, 
                                             double, 
                                             double,
                                             bool),
                        T* obj)
        {
            cb_ = boost::bind(fun, obj, _1, _2, _3, _4);
        }

    private:
        void imageCB(const sensor_msgs::Image& img);
        void callCB(sensor_msgs::Image::Ptr img_proc,
                    int                     x, 
                    int                     y,
                    bool                    green);

    };
}


#endif

