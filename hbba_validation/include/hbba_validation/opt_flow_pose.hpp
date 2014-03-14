#ifndef OPT_FLOW_POSE_HPP
#define OPT_FLOW_POSE_HPP

#include <image_tools/dense_optical_flow.hpp>
#include <imv_camera/imv_camera_processor.hpp>

namespace hbba_validation
{
    /// \brief A node producing poses pointing motion from the dense optical 
    /// flow of an IMV feed.
    /// 
    /// Processes the raw IMV in panorama mode, calculates the dense optical
    /// flow (based on image_tools::DenseOpticalFlow), and generate a pose in
    /// the direction of the most intense motion at (configurable) specific
    /// distance.
    /// 
    /// Topics:
    ///  - image_imv_raw: Raw IMV feed.
    ///  - motion_pose:   Pose of the most intense motion, in IMV's frame.
    ///  - ~proc_image:   Processed IMV image used as input.
    /// 
    /// Parameters:
    ///  - imv_width:     Width of the IMV processed image.
    ///                   Note that a 2:1 should be kept in panorama mode.
    ///                   Default: 320.
    ///  - imv_height:    Height of the IMV processed image.
    ///                   Note that a 2:1 should be kept in panorama mode.
    ///                   Default: 160.
    ///  - pose_dist:     Distance at which to generate the pose.
    ///                   Default: 1.0 m.
    ///
    class OptFlowPose: private image_tools::DenseOpticalFlow
    {
    private:
        ros::Subscriber                sub_image_;
        ros::Publisher                 pub_pose_;
        ros::Publisher                 pub_proc_image_;
        imv_camera::IMVCameraProcessor imv_proc_;

        int    imv_width_;
        int    imv_height_;
        double pose_dist_;

    public:
        /// \brief Constructor.
        ///
        /// \param n  Node handle for topics.
        /// \param np Node handle for parameters.
        OptFlowPose(ros::NodeHandle& n, ros::NodeHandle& np);

    private:
        void imageCB(const sensor_msgs::Image::ConstPtr& msg);
        void flowCB(const cv::Mat& flow);

    };
}


#endif

