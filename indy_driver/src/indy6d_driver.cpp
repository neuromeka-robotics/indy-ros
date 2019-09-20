#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "SocketHandler/IndyDCPSocket.h"
#include "TrajectoryDownloader/JointTrajectoryDownloader.h"

#define JOINT_DOF 6
#define DEGREE M_PI/180

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "indy6d_driver");
	ros::NodeHandle n;

	std::string robotName, ip;
	int port;
	double q[JOINT_DOF];
	sensor_msgs::JointState joint_state;

	// override IP/port with ROS params, if available
	ros::param::param<std::string>("robot_name", robotName, "");
	ros::param::param<std::string>("robot_ip_address", ip, SERVER_IP);
	ros::param::param<int>("~port", port, SERVER_PORT);

	// check for valid parameter values
	if (robotName.empty())
	{
		ROS_ERROR("No valid robot's name found.  Please set ROS 'robot_name' param");
		return false;
	}
	if (ip.empty())
	{
		ROS_ERROR("No valid robot IP address found.  Please set ROS 'robot_ip_address' param");
		return false;
	}
	if (port <= 0)
	{
		ROS_ERROR("No valid robot IP port found.  Please set ROS '~port' param");
		return false;
	}

	IndyDCPSocket indySocket;
	indySocket.init(robotName, ip, port);

	ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);

	JointTrajectoryDownloader jmotionInterface(indySocket, JOINT_DOF);
	jmotionInterface.init();
	// jmotionInterface.run();

	ros::Rate loop_rate(5);
	while (ros::ok())
	{
		if (indySocket.isWorking())
		{
			Data data;
			unsigned int len;

			indySocket.sendCommand(320, data, 0);
			indySocket.getFeedback(320, data, len);
			// printf("q: ");
			for (int i = 0; i < JOINT_DOF; i++)
			{
				q[i] = data.double6dArr[i];
				// printf("%f, ", q[i]);
			}
			// printf("\n");
		}

		joint_state.header.stamp = ros::Time::now();
		joint_state.name.resize(JOINT_DOF);
		joint_state.position.resize(JOINT_DOF);

		joint_state.name[0] = "joint0";
		joint_state.position[0] = q[0] * DEGREE;
		joint_state.name[1] = "joint1";
		joint_state.position[1] = q[1] * DEGREE;
		joint_state.name[2] = "joint2";
		joint_state.position[2] = q[2] * DEGREE;
		joint_state.name[3] = "joint3";
		joint_state.position[3] = q[3] * DEGREE;
		joint_state.name[4] = "joint4";
		joint_state.position[4] = q[4] * DEGREE;
		joint_state.name[5] = "joint5";
		joint_state.position[5] = q[5] * DEGREE;
		joint_pub.publish(joint_state);
		// jmotionInterface.updateJointState(joint_state);	

		ros::spinOnce();
		loop_rate.sleep();
	}

	indySocket.stop();

	return 0;
}