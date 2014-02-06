#include <hbba_validation/unlock_door.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "unlock_door");

    ros::NodeHandle n, np("~");
    hbba_validation::UnlockDoor node(n, np);

    ros::spin();

    return 0;
}

