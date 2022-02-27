#!/usr/bin/env python

from __future__ import print_function

import rospy
import tf2_ros
import tf2_geometry_msgs

#Gazebo imports
from gazebo_msgs.srv import SetModelState, SpawnModel
from gazebo_msgs.msg import ModelState
from gazebo_simulation_generator.srv import ObjectSpawn

from geometry_msgs.msg import PoseStamped, Point, Pose

CAMERA_FRAME = 'kinect2_depth_optical_frame'

use_gazebo = None

spawn_object_server = None
tf_buffer = None


def detect_object():
	try:
		detect = rospy.ServiceProxy('gazebo_simulation_generator/object_detect', ObjectSpawn)
		return detect()
	except rospy.ServiceException as e:
		print("gazebo_simulation_generator/object_detect Service call failed: %s" % e)


def spawn_object(object_to_spawn, object_name):
	global spawn_object_server
	global get_model
	global robot_odom_pose

	rospy.wait_for_service('spawn_object')

	position = object_to_spawn.pose.position

	try:
		spawn_object_server(object_name, position.x, position.y, position.z, 0, 0, 0, 0)
	except rospy.ServiceException as e:
		print("spawn_object Service call failed: %s" % e)


def pose_stamped(detected_object):
	input_pose = None

	if detected_object is not None:
		pose = Point(detected_object.x, detected_object.y, detected_object.z)
		input_pose = PoseStamped()
		input_pose.pose.position = pose
		input_pose.pose.orientation.x = 0
		input_pose.pose.orientation.y = 0
		input_pose.pose.orientation.z = 0.707108079859
		input_pose.pose.orientation.w = 0.707105482511
		input_pose.header.stamp = rospy.Time.now()
		input_pose.header.frame_id = CAMERA_FRAME
	
	return input_pose


def transform(object_pose, target_frame, source_frame_id=None):
	global tf_buffer
	if source_frame_id is None:
		source_frame_id = object_pose.header.frame_id
	transform_fetch_time = rospy.Time(0)
	timeout = rospy.Duration(1)

	while not rospy.is_shutdown():
		try:
			transform = tf_buffer.lookup_transform(target_frame, source_frame_id, transform_fetch_time)
		except Exception as e:
			rospy.Rate(2.0).sleep()
			continue

		return tf2_geometry_msgs.do_transform_pose(object_pose, transform)


def run():
		log('waiting for object_detect service')
		rospy.wait_for_service('gazebo_simulation_generator/object_detect')
		while not rospy.is_shutdown():
			camera_frame_object = detect_object()
			if camera_frame_object is None or not camera_frame_object.success:
				continue

			s = raw_input('Next item? (Press any key to continue)')
			log('Getting object detection data')
			
			camera_frame_pose = pose_stamped(camera_frame_object)
			if camera_frame_pose is not None:
				log('Doing transformation')
				base_footprint_frame = transform(camera_frame_pose, 'base_link')

				log('Spawning object')
				spawn_object(base_footprint_frame, camera_frame_object.objectName)



def log(msg, level='info'):
    if level is 'info':
        rospy.loginfo('[gazebo_spawn_client] ' + msg)


if __name__ == "__main__":
	print("hi")
	global MODEL_NAME
	rospy.init_node('spawn_object_client')
	tf_buffer = tf2_ros.Buffer()
	tf2_ros.TransformListener(tf_buffer)
	use_gazebo = rospy.get_param('~use_gazebo', False)

	if use_gazebo:
		print("hi1")
		MODEL_NAME = 'armadillo2'
	else:
		print("hi")
		MODEL_NAME = 'human_female_1_1'
	#Our own node for adding gazebo object
	spawn_object_server = rospy.ServiceProxy('spawn_object', ObjectSpawn)
	rospy.sleep(7)

	run()

	rospy.spin()
