
#ifndef ROBOT_MOVEMENT_CONFIG_PLUGIN_INTERFACE_H_
#define ROBOT_MOVEMENT_CONFIG_PLUGIN_INTERFACE_H_

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"

namespace robot_movement_config
{

	class PluginInterface
	{
		ros::Timer sendPathTimer;
		ros::Subscriber getCmdVelSub;
		ros::Subscriber getOdomRawSub;
		bool init;	/**< true if initialized. **/

		struct wheel_config {
			std::string link;
			ros::Publisher velocityPub;
			//ros::Publisher position;
			double scale_velocity; //multiply input with this to get radiant/s
			double scale_position; //multiply input with this to get radiant
			double radius;
		};

		std::vector<wheel_config> wheels;

	protected:
		ros::NodeHandle roshandle;

		struct wheel_velocity {
			std::string link;
			double velocity_mps;
		};

	private:

		/**call sendPath()
		 * @param event some time values
		 **/
		void sendPathCallback(const ros::TimerEvent& event);

		void getCmdVelCallback(const geometry_msgs::Twist& event);

		void getOdomRawCallback(const sensor_msgs::JointState& event);


	protected:

		bool isInit();

		bool addWheel(std::string link, std::string velocityPubTopic, double radius, double scale_velocity, double scale_position);

		virtual std::vector<wheel_velocity> getWheelVelFromCmdVel(geometry_msgs::Twist cmd_vel) = 0;

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
