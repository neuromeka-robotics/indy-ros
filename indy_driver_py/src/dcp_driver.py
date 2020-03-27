#!/usr/bin/python

import sys
import math
import json

from indydcp import indydcp_client
from indydcp import indy_program_maker

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Bool, Float64MultiArray, Empty
from moveit_msgs.msg import MoveGroupActionResult, DisplayRobotState
from geometry_msgs.msg import Pose

def rads2degs(rad_list):
    degs = [math.degrees(rad) for rad in rad_list]
    return degs

def degs2rads(deg_list):
    rads = [math.radians(deg) for deg in deg_list]
    return rads

class IndyROSConnector:
    def __init__(self, robot_ip, robot_name):
        self.robot_name = robot_name
        
        # Connect to Robot
        self.indy = indydcp_client.IndyDCPClient(robot_ip, robot_name)
        self.indy.connect()

        # Initialize ROS node
        rospy.init_node('indy_driver_py')
        self.rate = rospy.Rate(50) # hz

        self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.task_state_pub = rospy.Publisher('/indy/task_pos', Float64MultiArray, queue_size=10)

        self.query_joint_state_sub = rospy.Subscriber("/indy/query_joint_states", DisplayRobotState,  self.joint_state_callback, queue_size=10)
        self.plan_result_sub = rospy.Subscriber("/move_group/result", MoveGroupActionResult, self.plan_result_callback, queue_size=1)
        self.indy_execute_plan_sub = rospy.Subscriber("/indy/execute_plan", Bool, self.indy_execute_plan_callback, queue_size=1)
        self.stop_sub = rospy.Subscriber("/indy/stop_robot", Bool, self.stop_robot_callback, queue_size=1)
        
        # Initialize variable
        self.joint_pos_list = []
        self.is_executed = False

    def __del__(self):
        self.indy.disconnect()

    def joint_state_callback(self, msg):
        self.joint_pos_list = [msg.state.joint_state.position]
        
    def indy_execute_plan_callback(self, msg):
        if msg.data == True and self.is_executed == False:
            self.is_executed = True

    def plan_result_callback(self, msg):
        # download planned path from ros moveit
        self.joint_pos_list = []
        self.joint_pos_list = [p.positions for p in msg.result.planned_trajectory.joint_trajectory.points]

    def stop_robot_callback(self, msg):
        if msg.data == True:
            self.indy.stop_motion()

    def move_robot(self):
        prog = indy_program_maker.JsonProgramComponent(policy=0, resume_time=2)
            
        for pos in self.joint_pos_list:
            prog.add_joint_move_to(rads2degs(pos), vel=1, blend=5)
        
        json_string = json.dumps(prog.json_program)
        self.indy.set_and_start_json_program(json_string)
        self.joint_pos_list = []

    def publish_joint_state(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()

        if self.robot_name == 'NRMK-IndyRP2':
            joint_state_msg.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        else:
            joint_state_msg.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']

        joint_state_msg.position = degs2rads(self.indy.get_joint_pos())
        joint_state_msg.velocity = []
        joint_state_msg.effort = []

        self.joint_state_pub.publish(joint_state_msg)

    def publish_task_pos(self):
        task_pos_msg = Float64MultiArray()
        task_pos_msg.data = self.indy.get_task_pos()
        self.task_state_pub.publish(task_pos_msg)

    def run(self):
        while not rospy.is_shutdown():
            if self.is_executed:
                self.move_robot()
                self.is_executed = False
            else:
                self.publish_joint_state()
                self.publish_task_pos()

            self.rate.sleep()

def main():
    robot_ip = rospy.get_param("robot_ip_address")
    robot_name = rospy.get_param("robot_name")
    t = IndyROSConnector(robot_ip, robot_name)
    t.run()
                
if __name__ == '__main__':
    main()
        