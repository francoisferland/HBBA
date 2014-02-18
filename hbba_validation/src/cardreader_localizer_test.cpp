#include <hbba_validation/cardreader_localizer.hpp>
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>

namespace {

    boost::scoped_ptr<hbba_validation::CardreaderLocalizer> localizer_;

    void imageCB(const sensor_msgs::ImagePtr& img)
    {
        int x, y, c;
        c = localizer_->process(img, x, y);
        ROS_INFO("Red   c: %i, x: %i, y: %i", c, x, y);
        c = localizer_->process(img, x, y, true);
        ROS_INFO("Green c: %i, x: %i, y: %i", c, x, y);
    }

}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "cardreader_localizer");

    ros::NodeHandle n, np("~");
    localizer_.reset(new hbba_validation::CardreaderLocalizer());

    int t;
    np.param("threshold", t, 32);
    localizer_->threshold(t);

    ros::Subscriber sub_image_ = n.subscribe("image", 1, imageCB);

    ros::spin();

    return 0;
}
