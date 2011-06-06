
// in  : many actionlib clients
// out : one actionlib server
//
// TODO do the subscribers/publishers need a shutdown at the end?
//
// Multiple clients can connect to an action server, we just need
// to mux goal and cancel topics
//
// TODO for now the highest priority action client has the exclusive
// control of the server.
//
// Caveats
//   * the action client's wait_for_server will not work with this
//   mux. It uses the callerid of the last status message, and waits
//   until this caller advertises/subscribes all the topic. This mux
//   is between the server and the client for goal and cancel, so the
//   server will never connect to them.

#include <map>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <topic_tools/shape_shifter.h>
#include <topic_tools/parse.h>

#include <std_msgs/String.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalID.h>

using namespace topic_tools;

/*********************************************************\
|    A generic publisher                                  |
|                                                         |
| It doesn't need to know the message type by advance,    |
| The topic is advertised just before publishing the      |
| first message.                                          |
\*********************************************************/

// TODO assignation overload

class LazyPublisher
{
public:
	LazyPublisher(std::string topic = "") : topic(topic), advertised(false)
	{}
	void publish(const boost::shared_ptr<ShapeShifter const>& msg)
	{
		if (!advertised) {
			ros::NodeHandle n;
			pub = msg->advertise(n, topic, 5);
			advertised = true;
			// TODO don't understand why, but at first the publisher think
			// there is no subscribers, dropping the first message. Latching
			// seems not working too (will need a retry).
			while (pub.getNumSubscribers() == 0) {
				ros::Duration(0.1).sleep();
			}
		}
		pub.publish(msg);
	}
private:
	std::string topic;
	ros::Publisher pub;
	bool advertised;
};

/*********************************************************\
|    Action multiplexer                                   |
\*********************************************************/

/*
 * An interface is created for each client, but only one
 * is forwarded to the server.
 */
struct ClientInterface
{
	ros::Subscriber goal_sub;
	ros::Subscriber cancel_sub;
};

class ActionMux
{
public:

	ActionMux(std::string action_server) :
		selected_client(NULL),
		action_server(action_server)
	{}

	// TODO deletion
	void select(std::string action_client)
	{
		if(selected_client == NULL) { init(); }

		ClientInterface * client;
		
		std::map<std ::string, ClientInterface*>::iterator i;
		i = clients.find(action_client);
		if(i != clients.end()) {
			client = i->second;
		} else {
			client = init_client(action_client);
			clients[action_client] = client;
		}

		selected_client = client;
	}

	void select_cb(const std_msgs::StringConstPtr& str) {
		select(str->data); }

private:

	ClientInterface * selected_client;
	std::map<std ::string, ClientInterface*> clients;

	std::string action_server;

	ros::Subscriber status_sub;
	LazyPublisher goal_pub;
	ros::Publisher cancel_pub;

	/*
	 * Starts communications with the action server
	 *
	 * Only called when the first client is selected, to prevent the reception
	 * of message to the NULL client.
	 */
	void init() {
		ros::NodeHandle n;
		goal_pub = LazyPublisher(action_server + "/goal");
		cancel_pub = n.advertise<actionlib_msgs::GoalID>(
			action_server + "/cancel", 5);
		status_sub = n.subscribe<actionlib_msgs::GoalStatusArray>(
			action_server + "/status", 5,
			boost::bind(&ActionMux::status_cb, this, _1)
		);
	}

	/*
	 * Creates a client interface and start communications with it
	 */
	ClientInterface * init_client(std::string action_client) {
		ros::NodeHandle n;
		ClientInterface * client = new ClientInterface();
		client->goal_sub = n.subscribe<ShapeShifter>(
			action_client + "/goal", 5,
			boost::bind(&ActionMux::goal_cb, this, _1, client)
		);
		client->cancel_sub = n.subscribe<actionlib_msgs::GoalID>(
			action_client + "/cancel", 5,
			boost::bind(&ActionMux::cancel_cb, this, _1, client)
		);
		return client;
	}

	void status_cb(const actionlib_msgs::GoalStatusArrayConstPtr& msg) {
		//ROS_ERROR("received status for goal 0 %d", msg->status_list[0].status);
		 }

	/*
	 * Forwarding messages from the selected client to the server
	 */

	void goal_cb(
		const boost::shared_ptr<ShapeShifter const>& msg,
		ClientInterface* client
	) {
		if (client == selected_client) {
			goal_pub.publish(msg);
		}
	}

	void cancel_cb(
		const actionlib_msgs::GoalIDConstPtr& msg,
		ClientInterface* client
	) {
		if (client == selected_client) { cancel_pub.publish(msg); }
	}
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "action_mux");
	ros::NodeHandle n("~");
	
	std::string action_server;

	n.getParam("action_server", action_server);

	ActionMux mux = ActionMux(action_server);

	ros::Subscriber sub = n.subscribe<std_msgs::String>(
		"select", 1,
		boost::bind(&ActionMux::select_cb, mux, _1));

	ros::spin();

	return 0;
}
