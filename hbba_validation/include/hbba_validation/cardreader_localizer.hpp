#ifndef CARDREADER_LOCALIZER_HPP
#define CARDREADER_LOCALIZER_HPP

#include <sensor_msgs/Image.h>

namespace hbba_validation
{
    /// \brief A simple tool that localize cardreaders that unlock doors at the
    /// 3IT.
    ///
    /// The localization algorithm is entirely dependent on the visibility of
    /// the red LED on the device.
    /// It assumes that no other purely red light are in the same field of view.
    ///
    /// The process is simple: find the center of mass of pixels where 
    /// (R - G - B) > threshold.
    /// The result is in pixel coordinates, and further processing is needed to
    /// properly localize the cardreader, depending on the image's source.
    class CardreaderLocalizer
    {
    private:
        int threshold_;

    public:
        /// \brief Constructor.
        ///
        /// \param threshold The initial threshold used in the selection
        ///                  process.
        ///                  Default: 32.
        CardreaderLocalizer(const int threshold = 32);

        /// \brief Change the threshold value used to select searched pixels.
        void threshold(int t) { threshold_ = t; }

        /// \brief Processes the given image, return the possible location of a
        /// card reader.
        ///
        /// \param img The image to process, RGB format required.
        /// \param x   A reference to the output coordinate in X.
        /// \param y   A reference to the output coordinate in Y.
        /// \return The number of pixels used in the selection process.
        ///         A negative value means an error occured, while a value
        ///         that is too high (depends on the image resolution) suggests
        ///         that there might be more than one red light source.
        int process(const sensor_msgs::Image::ConstPtr& img, 
                          int&                          x, 
                          int&                          y) const;
    };
}

#endif

