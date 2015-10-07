
#include "robot_movement_config/plugin_interface.hpp"

namespace robot_movement_config
{
	PluginInterface::PluginInterface():init(false) {}

	void PluginInterface::initialize(ros::NodeHandle roshandle)
	{
		if (init) {
			ROS_WARN("[interface] plugin was already initialized. Nothing will be done.");
		} else {
			this->roshandle = roshandle;
			getCmdVelSub = roshandle.subscribe("cmd_vel", 2, &PluginInterface::getCmdVelCallback, this);

			getOdomRawSub = roshandle.subscribe("odom_raw", 2, &PluginInterface::getOdomRawCallback, this);

			jointStatePub = roshandle.advertise<sensor_msgs::JointState>("joint_states", 2);
			tfSenderTimer = roshandle.createTimer(ros::Duration(0.2), &PluginInterface::tfSenderCallback, this);

			last_pose.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
			last_pose.setRotation( tf::createQuaternionFromYaw(0) );

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


	void PluginInterface::tfSenderCallback(const ros::TimerEvent& event)
	{
		tf_bc.sendTransform(tf::StampedTransform(last_pose, event.current_real, "odom", "base_link"));
	}

	void PluginInterface::getCmdVelCallback(const geometry_msgs::Twist& event) {
		
		std::vector<wheel_data> vels=getWheelVelFromCmdVel(event);
		for (wheel_data vel : vels) {
			std::vector<wheel_config>::iterator it = wheels.begin();
			while (it->link.compare(vel.link) != 0 && it != wheels.end()) {it++;}
			if (it != wheels.end()) {
				std_msgs::Float64Ptr msg(new std_msgs::Float64);
				msg->data = vel.data / (it->radius * it->scale_velocity);
				it->velocityPub.publish(msg);
				ROS_DEBUG("set velocity for wheel on link \"%s\" to %.3lf rad/s (%.3lf m/s)", it->link.c_str(), msg->data * it->scale_velocity, vel.data);
			}
		}
	}

	void PluginInterface::getOdomRawCallback(const sensor_msgs::JointState& event) {
		sensor_msgs::JointStatePtr joint_state_msg(new sensor_msgs::JointState);

		joint_state_msg->header.frame_id = event.header.frame_id;
		joint_state_msg->header.stamp = event.header.stamp; //ros::Time::now();

		std::vector<wheel_data> odom_wheels;

		for (size_t i = 0; i < event.name.size(); i++) {
			joint_state_msg->name.push_back(event.name.at(i));
			std::vector<wheel_config>::iterator it = wheels.begin();
			bool found_wheel = true;
			while (it < wheels.end() && found_wheel) {
				if (it->link.compare(event.name.at(i)) != 0) {
					if (it == wheels.end() - 1) {
						found_wheel = false;
					} else {
						it++;
					}
				} else {
					break;
				}
			}
			if (found_wheel) {
				joint_state_msg->position.push_back(event.position.at(i) * it->scale_position);
				joint_state_msg->velocity.push_back(event.velocity.at(i) * it->scale_velocity);

				odom_wheels.push_back({it->link, event.position.at(i) * it->scale_position * it->radius});

				ROS_DEBUG("link \"%s\" at %.3lfm", it->link.c_str(), event.position.at(i) * it->scale_position * it->radius);
			} else {
				joint_state_msg->position.push_back(event.position.at(i));
				joint_state_msg->velocity.push_back(event.velocity.at(i));
				ROS_DEBUG("unknown link \"%s\"", event.name.at(i).c_str());
			}
			joint_state_msg->effort.push_back(event.effort.at(i));
		}

		jointStatePub.publish(joint_state_msg);

		if (odom_wheels.size() > 0) {
			geometry_msgs::Transform diff = getOdomDiff(odom_wheels);
			//ROS_INFO("%.3lf %.3lf %.3lf",diff.translation.x, diff.translation.y,tf::getYaw(diff.rotation) );
			double x_old = last_pose.getOrigin().x();
			double y_old = last_pose.getOrigin().y();
			double yaw_old = tf::getYaw(last_pose.getRotation());

			last_pose.setOrigin( tf::Vector3(
				x_old + cos(yaw_old) * diff.translation.x - sin(yaw_old) * diff.translation.y,
				y_old + sin(yaw_old) * diff.translation.x + cos(yaw_old) * diff.translation.y, 0.0) );
			last_pose.setRotation( tf::createQuaternionFromYaw(yaw_old + tf::getYaw(diff.rotation)) );
		}
	}

}
