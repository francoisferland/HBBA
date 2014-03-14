#include <hbba_validation/opt_flow_pose.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/imgproc/imgproc.hpp>

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

    last_header_ = msg->header;
    newImage(img);
}

void OptFlowPose::flowCB(const cv::Mat& flow)
{
    cv::Moments mu = cv::moments(flow);

    // TODO: Change 1e-3 as a bounded threshold (min/max amount of motion):
    if (mu.m00 < 1e-3) {
        return;
    }

    cv::Point pt(mu.m10 / mu.m00, mu.m01 / mu.m00);

    ROS_DEBUG_THROTTLE(1.0, "Current centroid: (%i, %i)", pt.x, pt.y);

    double pan, tilt;
    imv_proc_.orientationFromOutputPosition(pt.x, pt.y, pan, tilt);

    geometry_msgs::PoseStamped pose;
    pose.header = last_header_;

    // TODO: Make sure we're generating the pose in the correct orientation (X
    // in front, Y to the left ?)
    pose.pose.position.x    = pose_dist_ * cos(tilt) * cos(pan);
    pose.pose.position.y    = pose_dist_ * cos(tilt) * sin(pan);
    pose.pose.position.z    = pose_dist_ * sin(tilt);
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    pub_pose_.publish(pose);

}

