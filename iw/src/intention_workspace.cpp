#include <iw/intention_workspace.hpp>

int main (int argc, char **argv)
{
	ros::init(argc,argv, "iw");
	ros::NodeHandle n;

	iw::IntentionWorkspaceServer my_workspace(n);

	ros::spin();
	return 0;
}
