#include <hbba_validation/opt_flow_pose.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <opencv2/imgproc/imgproc.hpp>

using namespace hbba_validation;

OptFlowPose::OptFlowPose(ros::NodeHandle& n, ros::NodeHandle& np):
    image_tools::DenseOpticalFlow(np)
{
    np.param("pose_dist",    pose_dist_,        3.0);
    np.param("opt_flow_min", opt_flow_min_,  2500.0);
    np.param("opt_flow_max", opt_flow_max_, 50000.0);

    sub_image_ = n.subscribe("image_imv_raw", 10, &OptFlowPose::imageCB, this);

    pub_pose_ = n.advertise<geometry_msgs::PoseStamped>("motion_pose", 10);
    pub_proc_image_ = np.advertise<sensor_msgs::Image>("proc_image", 10);

    pub_m00_ = np.advertise<std_msgs::Float64>("opt_flow_sum", 10);

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

    if (pub_m00_.getNumSubscribers() > 0) {
        std_msgs::Float64 msg_m00;
        msg_m00.data = mu.m00;
        pub_m00_.publish(msg_m00);
    }

    if ((mu.m00 < opt_flow_min_) || (mu.m00 > opt_flow_max_)) {
        return;
    }

    cv::Point pt(mu.m10 / mu.m00, mu.m01 / mu.m00);

    ROS_DEBUG_THROTTLE(
        1.0, 
        "Current centroid, sum: (%i, %i), %f", 
        pt.x, 
        pt.y, 
        mu.m00);

    double pan, tilt;
    imv_proc_.orientationFromOutputPosition(pt.x, pt.y, pan, tilt);

    geometry_msgs::PoseStamped pose;
    pose.header = last_header_;

    pose.pose.position.x    = pose_dist_ * cos(tilt) * cos(-pan);
    pose.pose.position.y    = pose_dist_ * cos(tilt) * sin(-pan);
    pose.pose.position.z    = pose_dist_ * sin(tilt);
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    pub_pose_.publish(pose);

}

