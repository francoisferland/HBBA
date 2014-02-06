#ifndef CARDREADER_LOCALIZER_OPENNI_HPP
#define CARDREADER_LOCALIZER_OPENNI_HPP

#include "cardreader_localizer.hpp"
#include <openni_tools/rect_to_pose.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

namespace hbba_validation
{
    /// \brief A cardreader localizer with OpenNI-based full pose production.
    ///
    /// Approximate the cardreader's pose by averaging the depth in a square
    /// window around the detected LED's location.
    ///
    /// Topics:
    ///  - image: OpenNI RGB image.
    ///  - depth: OpenNI depth image.
    ///
    /// Parameters:
    ///  - threshold:   Threshold value for pixel detection, see
    ///                 CardreaderLocalizer for details.
    ///                 Default: 64.
    ///  - min_c:       Minimum detection pixel count for the red LED.
    ///                 Default: 6.
    ///  - max_c:       Maximum detection pixel count for the red LED.
    ///                 Too many pixels indicates more than a single red light
    ///                 source.
    ///                 Default: 30.
    ///  - led_radius:  Detection window radius in pixels [(-r, -r), (+r, +r)] 
    ///                 used to estimate the LED's distance.
    ///                 Default: 3.
    ///
    class CardreaderLocalizerOpenNI: 
        public openni_tools::RectToPose<CardreaderLocalizerOpenNI>
    {
    private:
        CardreaderLocalizer localizer_;

        int min_c_;
        int max_c_;
        int led_radius_;
        boost::function<void (const geometry_msgs::PoseStamped&)> valid_cb_;
        boost::function<void ()>                                  invalid_cb_;

    public:
        /// \brief Constructor.
        ///
        /// \param n  Node handle for topics.
        /// \param np Node handle for parameters.
        CardreaderLocalizerOpenNI(      ros::NodeHandle& n, 
                                  const ros::NodeHandle& np);

        /// \brief Register a valid detection callback.
        /// 
        /// Only called when the detection pixels count is between min_c and
        /// max_c.
        template <class T>
        void registerValidCB(void (T::*fun)(const geometry_msgs::PoseStamped&),
                             T* obj)
        {
            valid_cb_ = boost::bind(fun, obj, _1);
        }

        /// \brief Register an invalid detection callback.
        ///
        /// Only called when a received image did not produce a valid detection.
        template <class T>
        void registerInvalidCB(void (T::*fun)(), T* obj)
        {
            invalid_cb_ = boost::bind(fun, obj);
        }

        /// \brief Image callback used by RectToPose.
        void imageCB(const sensor_msgs::Image::ConstPtr& image,
                     const sensor_msgs::Image::ConstPtr& depth);
    };
}

#endif

