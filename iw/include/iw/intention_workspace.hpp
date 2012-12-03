#ifndef IW_HPP
#define IW_HPP

// ROS
#include <ros/ros.h>

// Messages and Services
#include <hbba_msgs/DesiresSet.h>
#include <hbba_msgs/AddDesires.h>
#include <hbba_msgs/RemoveDesires.h>
#include <hbba_msgs/SetDesireIntensity.h>
#include <std_srvs/Empty.h>

// STD headers
#include <string>
#include <vector>
#include <map>

typedef std::map<std::string, hbba_msgs::Desire> desire_map;

namespace iw
{

class IntentionWorkspaceServer 
{
	public:
	
	// Constructor for nodelets. It calls init() after
	IntentionWorkspaceServer() { /*		EMPTY	 	*/}

	// Constructor for nodes
	IntentionWorkspaceServer(ros::NodeHandle n)
	{
		// Init function
		init(n);
	}

	void init(ros::NodeHandle n)
	{
		// Copy the node handle
		nh = n;

		// Publish the desires
		pub_set = nh.advertise<hbba_msgs::DesiresSet>
					("desires_set", 1000 );
		
		// Service Server to manage desires
		server_add = nh.advertiseService("add_desires", 
				&IntentionWorkspaceServer::add_cb, this);
		server_rm = nh.advertiseService("rm_desires", 
				&IntentionWorkspaceServer::rm_cb, this);
		server_pub = nh.advertiseService("pub_desires", 
				&IntentionWorkspaceServer::pub_cb, this);
		server_intensity = nh.advertiseService(
				"set_desire_intensity",
			&IntentionWorkspaceServer::intensity_cb, this);

		desires.clear();
	}

	bool add_cb(hbba_msgs::AddDesires::Request &req, 
			 hbba_msgs::AddDesires::Response &res)
	{
		bool flag = true;

		// Get desires
		std::vector<hbba_msgs::Desire>::iterator it;

		// Sweep the vector
		for(it = req.desires.begin(); it != req.desires.end(); ++it)
		{
			// If it doesnt already exist
			if( desires.count(it->id) == 0 )
			{
				// Add the desire 
				desires.insert( std::pair<std::string, 
					hbba_msgs::Desire>( it->id, *it ) );
				
				ROS_INFO("IW : Adding new desire named %s",
					it->id.c_str());
			}
			else
			{
				flag = false;

				ROS_ERROR("IW : Could not add already "
					"existing %s desire.", 
						it->id.c_str());
			}
		}

		// Publish new set
		publish_desires();

		return flag;
	}	

	bool rm_cb(hbba_msgs::RemoveDesires::Request &req, 
			 hbba_msgs::RemoveDesires::Response &res)
	{
		bool flag = true;
		
		// Copy names
		std::vector<std::string>::iterator it;

		// Sweep
		for(it = req.ids.begin(); it != req.ids.end(); ++it)
		{
			// If it doesnt already exist, call ROS_ERROR
			if( desires.count(*it) == 0)
			{
				ROS_ERROR("IW : Could not remove "
					"non-existing %s desire.", 
						it->c_str());

				flag = false;
			}

			// Erase from the map
			desires.erase( desires.find(*it) );
		}
		
		// Publish new set
		publish_desires();

		return flag;
	}	

	void publish_desires()
	{
		// Create message
		hbba_msgs::DesiresSet msg;
		
		// Filter
		filter_desires();

		// Pass from map to vector of desires
		std::vector<hbba_msgs::Desire> des;
		desire_map::iterator it;
		for( it = desires.begin(); it != desires.end(); ++it)
		{
			des.push_back(it->second);
		}

		// Copy and publish
		msg.desires = des;
		pub_set.publish(msg);

	}

	void filter_desires()
	{
		desire_map::iterator it;
		for( it = desires.begin(); it != desires.end(); ++it)
		{
			// Erase desire with intensity == 0
			if(it->second.intensity < 0.001)
				desires.erase(it);
			else
			{
				// For desires with same utility
				desire_map::iterator it2;
				for( it2 = desires.begin(); 
					it2 != desires.end(); ++it2)
				{
					// Same utility & different name
					if( (it2->second.utility == 
						it->second.utility) &&
						(it2->second.id.compare(
						it->second.id) == 0 ))
					{
						// Erase the one with lowest
						// intensity
						if( it2->second.intensity >
						it->second.intensity)
						{
							desires.erase(it);
							break;
						}
						else
							desires.erase(it2);
					}
				}

			}

		}	
	}
	
	bool pub_cb(std_srvs::Empty::Request &req, 
			 std_srvs::Empty::Response &res)
	{
		publish_desires();
		return true;
	}

	bool intensity_cb(hbba_msgs::SetDesireIntensity::Request &req, 
			hbba_msgs::SetDesireIntensity::Response &res)
	{
		// if the id exist
		if( desires.count(req.id) != 0 )
		{
			ROS_INFO("IW : Changing intensity of %s desire "
				" from %f to %f", req.id.c_str(), 
				desires[req.id].intensity, req.value);

			desires[req.id].intensity = req.value;

			return true;
		}

		ROS_ERROR("IW : Can't change intensity of non-existing "
				"%s desire", req.id.c_str());
		
		// Return false if the desire doesn't exist
		return false;
	}
	private:
	ros::NodeHandle nh;

	// Desires Set publisher
	ros::Publisher pub_set;

	// Services Servers to manage the desires
	ros::ServiceServer server_add;
	ros::ServiceServer server_rm;
	ros::ServiceServer server_pub;
	ros::ServiceServer server_intensity;

	// Current list of desires mapped by their names
	// See typedef at line 19
	desire_map desires;

}; // End of class

} // End of namespace

#endif
