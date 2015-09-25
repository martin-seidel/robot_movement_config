
#include "robot_movement_config/plugin_interface.hpp"

namespace robot_movement_config
{
	PluginInterface::PluginInterface()
	{
		init = false;
	}

	void PluginInterface::initialize(ros::NodeHandle roshandle)
	{
		if (init) {
			ROS_WARN("[interface] plugin was already initialized. Nothing will be done.");
		} else {
			this->roshandle = roshandle;
			getCmdVelSub = roshandle.subscribe("cmd_vel", 2, &PluginInterface::getCmdVelCallback, this);

			getOdomRawSub = roshandle.subscribe("odom_raw", 2, &PluginInterface::getOdomRawCallback, this);

			//pathPub = roshandle.advertise<nav_msgs::Path>("path", 10);
			//sendPathTimer = roshandle.createTimer(ros::Duration(2.0), &PluginInterface::sendPathCallback, this);

			onInit(roshandle);
			init = true;
			ROS_INFO("[interface] plugin initialized.");

		}
	}

	int PluginInterface::onInit(ros::NodeHandle roshandle)
	{
		ROS_INFO("[interface] plugIn has no onInit function");
	}

	bool PluginInterface::isInit() {
		return init;
	}

	bool PluginInterface::addWheel(std::string link, std::string velocityPubTopic, double radius, double scale_velocity, double scale_position) {
		if (init) {
			ROS_WARN("adding a wheel-config after finishing the initialisation it's no allowed.");
			return false;
		} else {
			wheel_config temp;

	      temp.link = link;
	      temp.velocityPub = roshandle.advertise<std_msgs::Float64>(velocityPubTopic.c_str(), 2);
			temp.radius = radius;//read from urdf
	      temp.scale_velocity = scale_velocity;
	      temp.scale_position = scale_position;

			wheels.push_back(temp);
			ROS_INFO("added wheel on link \"%s\"", link.c_str());
			return true;
		}
	}


	void PluginInterface::sendPathCallback(const ros::TimerEvent& event)
	{
		ROS_INFO("[interface] timer");
	}

	void PluginInterface::getCmdVelCallback(const geometry_msgs::Twist& event) {

		std::vector<wheel_velocity> vels=getWheelVelFromCmdVel(event);
		for (wheel_velocity vel : vels) {
			std::vector<wheel_config>::iterator it = wheels.begin();
			while (it->link.compare(vel.link) != 0 && it != wheels.end()) {it++;}
			if (it != wheels.end()) {
				std_msgs::Float64Ptr msg(new std_msgs::Float64);
				msg->data = vel.velocity_mps / it->radius;
				it->velocityPub.publish(msg);
				ROS_DEBUG("set velocity for wheel on link \"%s\" to %.3lf rad/s", it->link.c_str(), msg->data);
			}
		}
	}

	void PluginInterface::getOdomRawCallback(const sensor_msgs::JointState& event) {
		ROS_INFO("[interface] timer");
	}

}
