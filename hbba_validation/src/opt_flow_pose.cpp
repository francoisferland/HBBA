#include <hbba_validation/opt_flow_pose.hpp>
#include <geometry_msgs/PoseStamped.h>

using namespace hbba_validation;

OptFlowPose::OptFlowPose(ros::NodeHandle& n, ros::NodeHandle& np):
    image_tools::DenseOpticalFlow(np)
{
    np.param("pose_dist", pose_dist_, 1.0);

    sub_image_ = n.subscribe("image_imv_raw", 10, &OptFlowPose::imageCB, this);

    pub_pose_ = n.advertise<geometry_msgs::PoseStamped>("motion_pose", 10);
    pub_proc_image_ = np.advertise<sensor_msgs::Image>("proc_image", 10);

    imv_proc_.mode(imv_camera::MODE_PERI);
    imv_proc_.position(imv_camera::POSITION_WALL);
    imv_proc_.width(320);
    imv_proc_.height(160);

    image_tools::DenseOpticalFlow::registerCB(&OptFlowPose::flowCB, this);
}

void OptFlowPose::imageCB(const sensor_msgs::Image::ConstPtr& msg)
{
    sensor_msgs::Image::Ptr img(new sensor_msgs::Image());
    imv_proc_.process(*msg, *img);

    if (pub_proc_image_.getNumSubscribers() != 0) {
        pub_proc_image_.publish(img);
    }

    newImage(img);
}

void OptFlowPose::flowCB(const cv::Mat& flow)
{
}

