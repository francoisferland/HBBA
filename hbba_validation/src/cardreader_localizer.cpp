#include <hbba_validation/cardreader_localizer.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>

using namespace hbba_validation;

CardreaderLocalizer::CardreaderLocalizer(const int threshold):
    threshold_(threshold)
{
}

int CardreaderLocalizer::process(
    const sensor_msgs::Image::ConstPtr& img, 
          int&                          x,
          int&                          y,
          bool                          green) const
{
    if (!sensor_msgs::image_encodings::isColor(img->encoding)) {
        ROS_ERROR("CardreaderLocalizer was given a non-color image.");
        return -1;
    } else if (sensor_msgs::image_encodings::bitDepth(img->encoding) != 8) {
        ROS_ERROR("CardreaderLocalizer was given an image "
                  "that isn't 8-bit/channel.");
        return -1;
    }

    
    const bool bgr = img->encoding == sensor_msgs::image_encodings::BGR8 ||
                     img->encoding == sensor_msgs::image_encodings::BGRA8;
    const int  ri  = bgr ? 2 : 0;
    const int  bi  = bgr ? 0 : 2;
    const int  gi  = 1;

    cv_bridge::CvImageConstPtr p_img = cv_bridge::toCvShare(img);
    const cv::Mat&             mat   = p_img->image; 

    cv::Mat rgb[3];
    cv::split(mat, rgb);

    cv::Mat eval = green ? rgb[gi] - rgb[ri] - rgb[bi]
                         : rgb[ri] - rgb[gi] - rgb[bi];

    x = y = 0;

    int sum = 0;
    int c   = 0;
    for (int i = 0; i < eval.rows; ++i) {
        for (int j = 0; j < eval.cols; ++j) {
            int v = eval.at<unsigned char>(i, j);
            if (v > threshold_) {
                ++c;
                sum += v;
                x   += v * j;
                y   += v * i;
            }
        }
    }

    if (c > 0) {
        x /= sum;
        y /= sum;
    }

    return c;
}
