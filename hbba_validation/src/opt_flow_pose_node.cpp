#include <hbba_validation/opt_flow_pose.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "opt_flow_pose");

    ros::NodeHandle n, np("~");
    hbba_validation::OptFlowPose node(n, np);

    ros::spin();

    return 0;
}

