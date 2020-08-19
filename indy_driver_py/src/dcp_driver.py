#!/usr/bin/python
#-*- coding: utf-8 -*-

import sys
import json

from indy_utils import indydcp_client
from indy_utils import indy_program_maker

import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Bool, Empty, Int32MultiArray
from geometry_msgs.msg import Pose
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryFeedback
import utils_transf

ROBOT_STATE = {
    0: "robot_ready", 
    1: "busy",
    2: "direct_teaching_mode",
    3: "collided",
    4: "emergency_stop",
}

class IndyROSConnector:
    def __init__(self, robot_ip, robot_name):
        self.robot_name = robot_name
        
        # Connect to Robot
        self.indy = indydcp_client.IndyDCPClient(robot_ip, robot_name)

        # Initialize ROS node
        rospy.init_node('indy_driver_py')
        self.rate = rospy.Rate(20) # hz

        # Publish current robot state
        self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.indy_state_pub = rospy.Publisher("/indy/status", GoalStatusArray, queue_size=10)
        self.control_state_pub = rospy.Publisher("/feedback_states", FollowJointTrajectoryFeedback, queue_size=1)

        # Subscribe desired joint position
        self.joint_execute_plan_sub = rospy.Subscriber("/joint_path_command", JointTrajectory, self.execute_plan_result_cb, queue_size=1)

        # Subscribe command
        self.execute_joint_state_sub = rospy.Subscriber("/indy/execute_joint_state", JointState, self.execute_joint_state_cb, queue_size=1)
        self.stop_sub = rospy.Subscriber("/stop_motion", Bool, self.stop_robot_cb, queue_size=1)
        self.set_motion_param_sub = rospy.Subscriber("/indy/motion_parameter", Int32MultiArray, self.set_motion_param_cb, queue_size=1)

        # Misc variable
        self.joint_state_list = []
        self.execute = False
        self.vel = 1
        self.blend = 5

    def __del__(self):
        self.indy.disconnect()

    def execute_joint_state_cb(self, msg):
        self.joint_state_list = [msg.position]

        if self.execute == False:
            self.execute = True

    def execute_plan_result_cb(self, msg):
        # download planned path from ros moveit
        self.joint_state_list = []
        if msg.points:
            self.joint_state_list = [p.positions for p in msg.points]
        else:
            self.indy.stop_motion()

        if self.execute == False:
            self.execute = True
    
    def stop_robot_cb(self, msg):
        if msg.data == True:
            self.indy.stop_motion()

    def set_motion_param_cb(self, msg):
        param_array = msg.data
        self.vel = param_array[0]
        self.blend = param_array[1]

    def move_robot(self):
        if self.joint_state_list:
            prog = indy_program_maker.JsonProgramComponent(policy=0, resume_time=2)
                
            for j_pos in self.joint_state_list:
                prog.add_joint_move_to(utils_transf.rads2degs(j_pos), vel=self.vel, blend=self.blend)
            
            json_string = json.dumps(prog.json_program)
            self.indy.set_and_start_json_program(json_string)
            self.joint_state_list = []

    def joint_state_publisher(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()

        if self.robot_name == 'NRMK-IndyRP2':
            joint_state_msg.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        else:
            joint_state_msg.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']

        joint_state_msg.position = utils_transf.degs2rads(self.indy.get_joint_pos())
        joint_state_msg.velocity = []
        joint_state_msg.effort = []

        control_state_msg = FollowJointTrajectoryFeedback()
        control_state_msg.header.stamp = rospy.Time.now()
        if self.robot_name == 'NRMK-IndyRP2':
            control_state_msg.joint_names = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        else:
            control_state_msg.joint_names = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']

        control_state_msg.actual.positions = utils_transf.degs2rads(self.indy.get_joint_pos())
        control_state_msg.desired.positions = utils_transf.degs2rads(self.indy.get_joint_pos())
        control_state_msg.error.positions = [0 for i in control_state_msg.joint_names]


        self.joint_state_pub.publish(joint_state_msg)
        self.control_state_pub.publish(control_state_msg)
        
    def robot_state_publisher(self):
        if self.current_robot_status['ready']:
            state_num = 0

        if self.current_robot_status['busy']:
            state_num = 1

        if self.current_robot_status['direct_teaching']:
            state_num = 2

        if self.current_robot_status['collision']:
            state_num = 3

        if self.current_robot_status['emergency']:
            state_num = 4

        status_msg = GoalStatusArray()
        status_msg.header.stamp = rospy.Time.now()

        status = GoalStatus()
        status.goal_id.stamp = rospy.Time.now()
        status.goal_id.id = ""
        status.status = state_num
        status.text = ROBOT_STATE[state_num]

        status_msg.status_list=[status]

        self.indy_state_pub.publish(status_msg)



    def run(self):
        self.indy.connect()
        while not rospy.is_shutdown():
            self.current_robot_status = self.indy.get_robot_status()
            self.joint_state_publisher()
            self.robot_state_publisher()

            if self.execute:
                self.execute = False
                if self.current_robot_status['busy']:
                    continue
                if self.current_robot_status['direct_teaching']:
                    continue
                if self.current_robot_status['collision']:
                    continue
                if self.current_robot_status['emergency']:
                    continue
                if self.current_robot_status['ready']:
                    self.move_robot()

        self.indy.disconnect()

def main():
    robot_ip = rospy.get_param("robot_ip_address")
    robot_name = rospy.get_param("robot_name")
    t = IndyROSConnector(robot_ip, robot_name)
    t.run()
                
if __name__ == '__main__':
    main()
        