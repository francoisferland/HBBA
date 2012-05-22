// Base class
#include <iw/intention_workspace.hpp>

// Nodelets
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace iw
{

class IntentionWorkspaceNodelet : public nodelet::Nodelet, 
				public iw::IntentionWorkspaceServer
{
	public:
	IntentionWorkspaceNodelet() {}

	virtual void onInit()
	{
		//initialisation fonction from EmotionManager
		IntentionWorkspaceServer::init(getNodeHandle());
	}

}; //end of class

} //end of namespace

PLUGINLIB_DECLARE_CLASS(iw, IntentionWorkspaceNodelet, 
			iw::IntentionWorkspaceNodelet, 
			nodelet::Nodelet)
