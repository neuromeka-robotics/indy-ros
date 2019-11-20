#!/usr/bin/env python
import time
import rospy
from math import pi, sin, cos, acos
import random
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

# Topics to write:
# type: std_msgs/Float64
# /indy7/joint0_position_controller/command

class Indy7JointMover(object):

	def __init__(self):
		rospy.init_node("indy7mover_demo", anonymous = True)
		rospy.loginfo("Indy7 JointMover Initialising...")

		self.pub_indy7_joint0_position = rospy.Publisher("/indy7/joint0_position_controller/command", Float64, queue_size=1)

		joint_states_topic_name = "/indy7/joint_states"
		rospy.Subscriber(joint_states_topic_name, JointState, self.indy7_joints_callback)
		indy7_joints_data = None
		while (indy7_joints_data is None):
			try:
				indy7_joints_data = rospy.wait_for_message(joint_states_topic_name, JointState, timeout=5)
			except:
				rospy.logwarn("Time out: " + str(joint_states_topic_name))
				pass
		self.indy7_joint_dictionary = dict(zip(indy7_joints_data.name, indy7_joints_data.position))

	def move_indy7_joint0(self, angle):
		angle_joint0 = Float64()
		angle_joint0.data = angle
		self.pub_indy7_joint0_position.publish(angle_joint0)

	def indy7_joints_callback(self, msg):
		self.indy7_joint_dictionary = dict(zip(msg.name, msg.position))

	def indy7_check_joint_value(self, joint_name, value, error=0.1):
		similar = self.indy7_joint_dictionary.get(joint_name) >= (value - error) and self.indy7_joint_dictionary.get(joint_name) <= (value + error)
		return similar	

	def convert_angle_to_unitary(self, angle):
		complete_rev = 2 * pi
		mod_angle = int(angle / complete_rev)
		clean_angle = angle - complete_rev * mod_angle

		if clean_angle < 0:
			clean_angle += 2 * pi
		return clean_angle

	def assertAlmostEqualAngles(self, x, y):
		c2 = (sin(x) - sin(y))**2 + (cos(x) - cos(y))**2
		angle_diff = acos((2.0 - c2)/2.0)
		return angle_diff

	def indy7_check_continuous_joint_value(self, joint_name, value, error=0.1):
		joint_reading = self.indy7_joint_dictionary.get(joint_name)
		clean_joint_reading = self.convert_angle_to_unitary(angle=joint_reading)
		clean_value = self.convert_angle_to_unitary(angle=value)
		diff_angle = self.assertAlmostEqualAngles(clean_joint_reading, clean_value)
		similar = diff_angle <= error
		return similar

	def indy7_movement_exec(self, angle0):
		check_rate = 5.0
		similar_angle0 = False
		rate = rospy.Rate(check_rate)
		while not (similar_angle0):
			self.move_indy7_joint0(angle0)
			similar_angle0 = self.indy7_check_continuous_joint_value(joint_name="joint0", value=angle0)
			rate.sleep()

	def indy7_move_randomly(self):
		angle0 = random.uniform(-1.5, 1.5)
		self.indy7_movement_exec(angle0)

	def movement_random_loop(self):
		rospy.loginfo("Start Moving Indy7...")
		while not rospy.is_shutdown():
			self.indy7_move_randomly()

if __name__ == "__main__":
	indy7_jointmove_obj = Indy7JointMover()
	indy7_jointmove_obj.movement_random_loop()