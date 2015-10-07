
#ifndef ROBOT_MOVEMENT_CONFIG_PLUGIN_INTERFACE_H_
#define ROBOT_MOVEMENT_CONFIG_PLUGIN_INTERFACE_H_

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Transform.h"
#include <tf/transform_broadcaster.h>
#include <math.h>

namespace robot_movement_config
{

	class PluginInterface
	{
		ros::Timer tfSenderTimer;
		ros::Subscriber getCmdVelSub;
		ros::Subscriber getOdomRawSub;
		ros::Publisher jointStatePub;
		bool init;	/**< true if initialized. **/
		tf::TransformBroadcaster tf_bc;
		tf::Transform last_pose;

		struct wheel_config {
			std::string link;
			ros::Publisher velocityPub;
			//ros::Publisher position;
			double scale_velocity; //multiply input with this to get radiant/s
			double scale_position; //multiply input with this to get radiant
			double radius; //urdf!
		};

		std::vector<wheel_config> wheels;

	protected:
		ros::NodeHandle roshandle;

		struct wheel_data {
			std::string link;
			double data;
		};

	private:

		/**
		 * @param event some time values
		 **/
		void tfSenderCallback(const ros::TimerEvent& event);

		void getCmdVelCallback(const geometry_msgs::Twist& event);

		void getOdomRawCallback(const sensor_msgs::JointState& event);


	protected:

		bool isInit();

		bool addWheel(std::string link, std::string velocityPubTopic, double radius, double scale_velocity, double scale_position);

		virtual std::vector<wheel_data> getWheelVelFromCmdVel(geometry_msgs::Twist cmd_vel) = 0;

		virtual geometry_msgs::Transform getOdomDiff(std::vector<wheel_data> current_position) = 0;

		/** initialize #init
		 **/
		PluginInterface();

		/**In this function the plugIn can do some initialisation
		 * depending on a ROS NodeHandle.
		 * @param roshandle a valid ROS NodeHandle
		 * @return error code
		 **/
		virtual int onInit(ros::NodeHandle roshandle);



	public:
		/**Initalize the global variables.
		 * @param roshandle a valid ROS NodeHandle
		 **/
		void initialize(ros::NodeHandle roshandle);



		virtual ~PluginInterface() {}	/**< Intentionally left empty **/
	};
};
#endif //ROBOT_MOVEMENT_CONFIG_PLUGIN_INTERFACE_H_
