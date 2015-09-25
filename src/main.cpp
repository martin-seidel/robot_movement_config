#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include "robot_movement_config/plugin_interface.hpp"

boost::shared_ptr<robot_movement_config::PluginInterface> plugin_node;
/*#include <nodelet/nodelet.h>

#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <dynamic_reconfigure/server.h>
#include <plugin_demo_main/plugin_demoConfig.h>

boost::shared_ptr<plugin_demo_interface_namespace::PathPlanner> plugin_node;
boost::shared_ptr<nodelet::Nodelet> plugin_nodelet;
bool change_plugin;

std::string plugin_name;
bool use_shared_pointer;
ros::Publisher setStartPub;
double delta_x, delta_y, delta_theta;*/

/*void getStartCallback(const geometry_msgs::PoseWithCovarianceStamped event)
{
	if (use_shared_pointer)	//shared pointer demo
	{
		geometry_msgs::Pose2DPtr msg(new geometry_msgs::Pose2D);
		msg->x = event.pose.pose.position.x;
		msg->y = event.pose.pose.position.y;
		msg->theta = plugin_demo_interface_namespace::PathPlanner::getYawFromQuat(event.pose.pose.orientation);
		setStartPub.publish(msg);
		msg->x += delta_x;
		msg->y += delta_y;
		msg->theta += delta_theta;
	} else {
		geometry_msgs::Pose2D msg;
		msg.x = event.pose.pose.position.x;
		msg.y = event.pose.pose.position.y;
		msg.theta = plugin_demo_interface_namespace::PathPlanner::getYawFromQuat(event.pose.pose.orientation);
		setStartPub.publish(msg);
		msg.x += delta_x;
		msg.y += delta_y;
		msg.theta += delta_theta;
	}

}

void reconfigureCallback(plugin_demo_main::plugin_demoConfig &config, uint32_t level)
{
	use_shared_pointer = config.use_shared_pointer;
	delta_x = config.delta_x;
	delta_y = config.delta_y;
	delta_theta = config.delta_theta;

	if (config.plugin != plugin_name)
	{
		plugin_name = config.plugin;
		change_plugin = true;
	}
}*/

int loadPlugin(pluginlib::ClassLoader<robot_movement_config::PluginInterface> *plug_in_loader, ros::NodeHandle *roshandle, std::string plugin_name)
{
	ROS_INFO("[main] starting plugin \"%s\"", plugin_name.c_str());
	try
	{
		plugin_node.reset();
		plugin_node = plug_in_loader->createInstance(plugin_name.c_str());
		plugin_node->initialize(*roshandle);
	} catch(pluginlib::PluginlibException& ex) {
		ROS_FATAL("[main] failed to load. Error: \"%s\"\nGoodbye.", ex.what());
		return 1;
	}
	ROS_INFO("[main] module loaded");
	return 0;
}

/*int loadNodelet(pluginlib::ClassLoader<nodelet::Nodelet> *nodelet_loader, std::string nodelet_name)
{
	ROS_INFO("[main] starting nodelet \"%s\"", nodelet_name.c_str());
	try
	{
		plugin_nodelet.reset();
		plugin_node.reset();
		plugin_nodelet = nodelet_loader->createInstance(nodelet_name.c_str());

		ros::M_string remapping_args;
		remapping_args["set_target"] = "/set_target";
		remapping_args["set_start"] = "/set_start";
		remapping_args["get_start"] = "/get_start";
		remapping_args["path"] = "/path";
		std::vector<std::string> my_argv;
		my_argv.push_back("with_main");
		plugin_nodelet->init(nodelet_name, remapping_args, my_argv);

	} catch(pluginlib::PluginlibException& ex) {
		ROS_FATAL("[main] failed to load. Error: \"%s\"\nGoodbye.", ex.what());
		return 1;
	}
	ROS_INFO("[main] nodelet-module loaded");
	return 0;
}*/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robot_movement_config");
	ros::NodeHandle roshandle;

	pluginlib::ClassLoader<robot_movement_config::PluginInterface> plug_in_loader("robot_movement_config", "robot_movement_config::PluginInterface");

	loadPlugin(&plug_in_loader, &roshandle, "robot_movement_config::DifferentialSteering");

	ros::spin();

	plugin_node.reset();

	/*dynamic_reconfigure::Server<plugin_demo_main::plugin_demoConfig> reconfServer;
	dynamic_reconfigure::Server<plugin_demo_main::plugin_demoConfig>::CallbackType f;
	f = boost::bind(&reconfigureCallback, _1, _2);
	reconfServer.setCallback(f);

	ros::Subscriber newStartSub;
	newStartSub = roshandle.subscribe("get_start", 10, &getStartCallback);
	setStartPub = roshandle.advertise<geometry_msgs::Pose2D>("set_start", 10);
	use_shared_pointer = false;

	//the plugin_name should include the namespace
	ros::param::param<std::string>("~plugin", plugin_name, "plugin_demo_plugins_namespace::Direct");
	change_plugin = false;

	pluginlib::ClassLoader<plugin_demo_interface_namespace::PathPlanner> plug_in_loader("plugin_demo_interface", "plugin_demo_interface_namespace::PathPlanner");
	pluginlib::ClassLoader<nodelet::Nodelet> nodelet_loader("nodelet", "nodelet::Nodelet");

	if (plugin_name.rfind("Nodelet") == std::string::npos)
	{
		loadPlugin(&plug_in_loader, &roshandle, plugin_name);
		plugin_node->initialize(roshandle);
	} else {
		loadNodelet(&nodelet_loader, plugin_name);
	}

	ros::Rate freq(8.0);

	while (ros::ok())
	{
		if (change_plugin) {
			if (plugin_name.rfind("Nodelet") == std::string::npos)
			{
				geometry_msgs::Pose2D start, target;
				if ( plugin_node.unique() ) plugin_node->getPoints(&start, &target);
				loadPlugin(&plug_in_loader, &roshandle, plugin_name);
				if ( plugin_node.unique() ) plugin_node->setPoints(&start, &target);
			} else {
				//load & store e.g. as service, which is calling getPoints() and setPoints() inside the nodelet
				loadNodelet(&nodelet_loader, plugin_name);
			}
			change_plugin = false;
		}
		ros::spinOnce();
		freq.sleep();
	}

	plugin_node.reset();
	plugin_nodelet.reset();*/

	return 0;
}
