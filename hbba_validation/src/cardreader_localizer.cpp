#include <hbba_validation/cardreader_localizer.hpp>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>

using namespace hbba_validation;

CardreaderLocalizer::CardreaderLocalizer(const int threshold):
    threshold_(threshold)
{
    ros::NodeHandle np("~");
    pub_found_ = np.advertise<sensor_msgs::Image>("led_found", 1);
}

bool CardreaderLocalizer::prepareImg(
    const sensor_msgs::Image::ConstPtr &img,
          cv_bridge::CvImageConstPtr   &p_img,
          cv::Mat                     (&rgb)[3],
          int                          &ri,
          int                          &gi,
          int                          &bi) const
{
    if (!sensor_msgs::image_encodings::isColor(img->encoding)) {
        ROS_ERROR("CardreaderLocalizer was given a non-color image.");
        return false;
    } else if (sensor_msgs::image_encodings::bitDepth(img->encoding) != 8) {
        ROS_ERROR("CardreaderLocalizer was given an image "
                  "that isn't 8-bit/channel.");
        return false;
    }

    
    const bool bgr = img->encoding == sensor_msgs::image_encodings::BGR8 ||
                     img->encoding == sensor_msgs::image_encodings::BGRA8;
    ri  = bgr ? 2 : 0;
    bi  = bgr ? 0 : 2;
    gi  = 1;

    p_img = cv_bridge::toCvShare(img);

    const cv::Mat& mat = p_img->image; 
    cv::split(mat, rgb);

    return true;
}

int CardreaderLocalizer::evalMat(const cv::Mat &m,
                                       int     &x,
                                       int     &y) const
 {
    int c   = 0;
    int sum = 0;

    x = y = 0;

    for (int i = 0; i < m.rows; ++i) {
        for (int j = 0; j < m.cols; ++j) {
            int v = m.at<unsigned char>(i, j);
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

int CardreaderLocalizer::process(
    const sensor_msgs::Image::ConstPtr& img, 
          int&                          x,
          int&                          y,
          bool                          green) const
{
    cv_bridge::CvImageConstPtr p_img;
    int ri, gi, bi;
    cv::Mat rgb[3];
    if (!prepareImg(img, p_img, rgb, ri, gi, bi)) {
        return -1;
    }

    cv::Mat eval = green ? rgb[gi] - rgb[ri] - rgb[bi]
                         : rgb[ri] - rgb[gi] - rgb[bi];

    int c = evalMat(eval, x, y);

    if (c > 0) {
        if (pub_found_.getNumSubscribers() > 0) {
            cv_bridge::CvImage found_cv(*p_img);
            cv::circle(found_cv.image, 
                       cv::Point(x, y), 
                       5, 
                       cv::Scalar(255, 255, 255));
            pub_found_.publish(found_cv.toImageMsg());
        }
    }

    return c;
}

bool CardreaderLocalizer::processBoth(
    const sensor_msgs::Image::ConstPtr& img,
          int&                          r_x,
          int&                          r_y,
          int&                          r_c,
          int&                          g_x,
          int&                          g_y,
          int&                          g_c) const
{
    cv_bridge::CvImageConstPtr p_img;
    int ri, gi, bi;
    cv::Mat rgb[3];
    if (!prepareImg(img, p_img, rgb, ri, gi, bi)) {
        return false;
    }

    cv::Mat m_r = rgb[ri] - rgb[gi] - rgb[bi];
    cv::Mat m_g = rgb[gi] - rgb[ri] - rgb[bi];

    r_c = evalMat(m_r, r_x, r_y);
    g_c = evalMat(m_g, g_x, g_y);

    return true;
}
