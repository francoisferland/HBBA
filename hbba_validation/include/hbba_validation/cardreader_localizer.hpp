#ifndef CARDREADER_LOCALIZER_HPP
#define CARDREADER_LOCALIZER_HPP

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>

namespace hbba_validation
{
    /// \brief A simple tool that localize cardreaders that unlock doors at the
    /// 3IT.
    ///
    /// The localization algorithm is entirely dependent on the visibility of
    /// the red or green LED on the device.
    /// It assumes that no other purely red light are in the same field of view.
    ///
    /// The process is simple: find the center of mass of pixels where 
    /// (R - G - B) > threshold for the red LED, and (G - R - B) > threshold for
    /// the green one.
    /// The result is in pixel coordinates, and further processing is needed to
    /// properly localize the cardreader, depending on the image's source.
    ///
    /// Also publishes a "~led_found" topic for diagnostic purposes.
    /// Only publishes an image if at least one pixel passed the threshold.
    ///
    class CardreaderLocalizer
    {
    private:
        ros::Publisher pub_found_;

        int            threshold_;

    public:
        /// \brief Constructor.
        ///
        /// \param threshold The initial threshold used in the selection
        ///                  process.
        ///                  Default: 32.
        CardreaderLocalizer(const int threshold = 32);

        /// \brief Change the threshold value used to select searched pixels.
        void threshold(int t) { threshold_ = t; }

        /// \brief Processes the given image for a single LED, return the 
        ///possible location of a card reader.
        ///
        /// \param img   The image to process, RGB format required.
        /// \param x     A reference to the output coordinate in X.
        /// \param y     A reference to the output coordinate in Y.
        /// \param green Process for a green LED instead of red.
        /// \return The number of pixels used in the selection process.
        ///         A negative value means an error occured, while a value
        ///         that is too high (depends on the image resolution) suggests
        ///         that there might be more than one red light source.
        int process(const sensor_msgs::Image::ConstPtr& img, 
                          int&                          x, 
                          int&                          y,
                          bool                          green = false) const;

        /// \brief Process the given image for both LED colors.
        ///
        /// \param r_x   A reference to the output coordinate in X for the red
        ///              LED.
        /// \param r_y   A reference to the output coordinate in Y for the red
        ///              LED.
        /// \param r_c   The number of selected pixels for the red LED.
        /// \param g_x   A reference to the output coordinate in X for the green
        ///              LED.
        /// \param g_y   A reference to the output coordinate in Y for the green
        ///              LED.
        /// \param g_c   The number of selected pixels for the green LED.
        /// \return      False if an error occured.
        bool processBoth(const sensor_msgs::Image::ConstPtr& img,
                               int&                          r_x,
                               int&                          r_y,
                               int&                          r_c,
                               int&                          g_x,
                               int&                          g_y,
                               int&                          g_c) const;

    private:
        bool prepareImg(const sensor_msgs::Image::ConstPtr  &img,
                              cv_bridge::CvImageConstPtr   &p_img,
                              cv::Mat                     (&rgb)[3],
                              int                          &ri,
                              int                          &gi,
                              int                          &bi) const;

        int evalMat(const cv::Mat &m,
                          int     &x,
                          int     &y) const;
        
    };
}

#endif

