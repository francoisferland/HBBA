#include <hbba_validation/tour_guide.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tour_guide_node");

    ros::NodeHandle n, np("~");
    hbba_validation::TourGuide node(n, np);

    ros::spin();

    return 0;
}

