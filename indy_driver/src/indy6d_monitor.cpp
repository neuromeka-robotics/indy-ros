#include <string>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "SocketHandler/IndyDCPSocket.h"

#define JOINT_DOF 6
#define DEGREE M_PI/180

int main(int argc, char ** argv)
{
	std::string robotNamespace, robotName, ip;
	int port;
	double q[JOINT_DOF];

	std::string nodeName = "indy6d_monitor";
	ros::init(argc, argv, nodeName);
	ros::NodeHandle n;

	// override IP/port with ROS params, if available
	ros::param::param<std::string>("robot_ns", robotNamespace, "");
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

	//ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);
	std_msgs::Float64 jointCommands[JOINT_DOF];
	ros::Publisher jointPubs[JOINT_DOF];
	jointPubs[0] = n.advertise<std_msgs::Float64>("/" + robotNamespace + "/joint0_position_controller/command", 10);
	jointPubs[1] = n.advertise<std_msgs::Float64>("/" + robotNamespace + "/joint1_position_controller/command", 10);
	jointPubs[2] = n.advertise<std_msgs::Float64>("/" + robotNamespace + "/joint2_position_controller/command", 10);
	jointPubs[3] = n.advertise<std_msgs::Float64>("/" + robotNamespace + "/joint3_position_controller/command", 10);
	jointPubs[4] = n.advertise<std_msgs::Float64>("/" + robotNamespace + "/joint4_position_controller/command", 10);
	jointPubs[5] = n.advertise<std_msgs::Float64>("/" + robotNamespace + "/joint5_position_controller/command", 10);

	ros::Rate loop_rate(20);
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
				//jointCommands[i].header.stamp = ros::Time::now();
				jointCommands[i].data = q[i] * DEGREE;
				jointPubs[i].publish(jointCommands[i]);
			}
			// printf("\n");
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	indySocket.stop();

	return 0;
}