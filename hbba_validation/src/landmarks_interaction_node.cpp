#include <hbba_validation/landmarks_interaction.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "landmarks_interaction_node");
    
    ros::NodeHandle n, np("~");
    hbba_validation::LandmarksInteraction node(n, np);

    ros::spin();

    return 0;

}

