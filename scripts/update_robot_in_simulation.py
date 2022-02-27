#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_simulation_generator.srv import ObjectSpawn
from gazebo_msgs.srv import SetModelState
from nav_msgs.msg import Odometry

robot_odom_pose = None
robot_inited = False
spawn_object_server = None
set_model = None

MODEL_NAME = 'human_female_1_1'
INFO = 'info'

def save_odom_and_update_robot_state(msg):
	global robot_odom_pose

	robot_odom_pose = msg.pose.pose
	if not robot_inited:
		spawn_robot()
	update_robot_model()

def spawn_robot():
	global robot_inited
	log('waiting for service: spawn_object')
	rospy.wait_for_service('spawn_object')
	log('waiting for service: /gazebo/set_model_state')
	rospy.wait_for_service('/gazebo/set_model_state')

	position = robot_odom_pose.position
	orientation = robot_odom_pose.orientation
	try:
		spawn_object_server(MODEL_NAME, position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w)
	except rospy.ServiceException as e:
		print("robot spawn_object Service call failed: %s" % e)
	robot_inited = True

def update_robot_model():
	global robot_odom_pose
	state_msg = ModelState()
	state_msg.model_name = MODEL_NAME
	state_msg.pose = robot_odom_pose
	try:
		res = set_model(state_msg)
	except rospy.ServiceException, e:
		print("/gazebo/set_model_state Service call failed: %s" % e)

def log(msg, level=INFO):
    if level is 'info':
        rospy.loginfo('[update_robot_in_simulation]- ' + msg)

if __name__ == "__main__":
	rospy.init_node('update_robot_in_simulation')
	spawn_object_server = rospy.ServiceProxy('spawn_object', ObjectSpawn)
	set_model = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
	odom_sub = rospy.Subscriber('/mobile_base_controller/odom', Odometry, save_odom_and_update_robot_state)
	rospy.spin()
	

