#ifndef JOINT_TRAJECTORY_DOWNLOADER_H
#define JOINT_TRAJECTORY_DOWNLOADER_H

#include <map>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <industrial_msgs/CmdJointTrajectory.h>
#include "industrial_msgs/StopMotion.h"
#include "sensor_msgs/JointState.h"
#include "simple_message/messages/joint_traj_pt_message.h"
#include "trajectory_msgs/JointTrajectory.h"

#include "control_msgs/FollowJointTrajectoryFeedback.h"

#include "../SocketHandler/IndyDCPSocket.h"

using industrial::joint_traj_pt_message::JointTrajPtMessage;

class JointTrajectoryDownloader
{

public:
	JointTrajectoryDownloader(IndyDCPSocket & indySocket, int joint_dof);
	~JointTrajectoryDownloader();

	void trajectoryStop();

	bool init();
	void run();

	void updateJointState(const sensor_msgs::JointState & curJointState);
	
	void jointStateCB(const sensor_msgs::JointStateConstPtr &msg);
	bool stopMotionCB(industrial_msgs::StopMotion::Request &req,
                                    industrial_msgs::StopMotion::Response &res);
	void jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg);
	bool jointTrajectoryCB(industrial_msgs::CmdJointTrajectory::Request &req,
                         industrial_msgs::CmdJointTrajectory::Response &res);

private:
	IndyDCPSocket & _indySocket;
	ros::NodeHandle node_;
	ros::Subscriber sub_cur_pos_;  // handle for joint-state topic subscription
	ros::Subscriber sub_joint_trajectory_; // handle for joint-trajectory topic subscription
	ros::ServiceServer srv_joint_trajectory_;  // handle for joint-trajectory service
	ros::ServiceServer srv_stop_motion_;   // handle for stop_motion service
	ros::Publisher pub_joint_control_state_;
	sensor_msgs::JointState cur_joint_pos_;  // cache of last received joint state


	int _joint_dof;
};

#endif /* JOINT_TRAJECTORY_DOWNLOADER_H */