#!/usr/bin/env python
import rospy
import sensor_msgs.point_cloud2 as pc2
from roslib import message

from sensor_msgs.msg import PointCloud2
from gazebo_simulation_generator.srv import ObjectSpawn, ObjectSpawnResponse
from geometry_msgs.msg import PoseStamped, Point
from obj_detector.msg import Detection_msg, Pose

INFO = 'info'

WAITING_LIST_SIZE = 100
data = None
yolo_data = []

#database
database = []
delta = 0.25

PC2_TOPIC = '/kinect2/qhd/points'
pc2_data = None

tf_buffer = None
first_time = True

#get this param from the launch input.
#true for yolo, false for simple colour object detect
use_yolo = None


class PointCloud:
    def __init__(self):
        rospy.Subscriber(PC2_TOPIC, PointCloud2, self.listener)

    def listener(self, msg):

				self.data = msg


def map_name(name):
	if name == 'diningtable':
		return 'small_table'
	return name
		

def in_database(object_name, x, y):
	global delta
	rospy.logdebug('[in_database]: database: %s', database)
	for object_tuple in database:
		if object_tuple[0] == object_name:
			if x < object_tuple[1] + delta and x > object_tuple[1] - delta:
				if y < object_tuple[2] + delta and y > object_tuple[2] - delta:
					return True
	return False


def insert_to_db(object_name, x, y, z):
  database.append((object_name, x, y, z))

def object_detect_listener(pose_data):
	global first_time
	#Let the PC data some time to come up.
	if first_time:
		first_time = False
		rospy.sleep(3.5)

	pose_data.class_id = map_name(pose_data.class_id)
	
	#How many object we save in the waiting list
	if len(yolo_data) < WAITING_LIST_SIZE:
		position = fetch_pc_coordinates(pose_data)
		#In case PC2 failed to find the x,y,z - Dont save the current object.
		if not position:
			return
		x, y, z = position
		if not in_database(pose_data.class_id, x, y):
			insert_to_db(pose_data.class_id, x, y, z)
			position.insert(0, pose_data.class_id)
			yolo_data.append(tuple(position))


def handle_yolo_detect_request(_data):
	global yolo_data
	while not yolo_data:
		log('waiting for yolo_data')
		rospy.sleep(1)

	rospy.logdebug('[handle_yolo_detect_request]: Handling yolo request')
	item_tuple = yolo_data.pop(0)
	name, x, y, z = item_tuple
	log("Sent object name: %s " %(name))
	return ObjectSpawnResponse(True, name, x, y, z, 0, 0, 0, 0)


def simple_object_detect_listener(pose_data):
	global data
	data = pose_data

def handle_detect_request(_data):
	global data
	try:
		while not data:
			log('waiting for data')
			rospy.sleep(1)
			
		position = data.pose.position
		orientation = data.pose.orientation
		# Reset it for the next time
		data = None

		return ObjectSpawnResponse(True, 'can_coke', position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w)
	except rospy.ServiceException as e:
		return ObjectSpawnResponse(False, 'failed', 0, 0, 0, 0, 0, 0, 0)


def fetch_pc_coordinates(msg):
	global pc2_data
	
	list_of_tuple_position = []
	x = msg.pose.x_center
	y = msg.pose.y_center

	[u, v] = [int(x), int(y)]
	list_of_tuple_position = list(pc2.read_points(pc2_data.data, ('x', 'y', 'z'), skip_nans=True, uvs=[[u, v]]))
	if not list_of_tuple_position:
		return

	return list(list_of_tuple_position[0])


def log(msg, level='info'):
	if level is 'info':
		rospy.loginfo('[object_detect] ' + msg)


if __name__ == '__main__':
	global pc2_data
	rospy.init_node('gazebo_generator_object_detect', anonymous=True, log_level=rospy.INFO)
	use_yolo = rospy.get_param('~use_yolo', True)
	
	if use_yolo:
		# Yolo4
		log('Starting object detecion with yolo4')
		pc2_data = PointCloud()
		rospy.Subscriber('/yolo4_result/detections', Detection_msg, object_detect_listener)
		service = rospy.Service('gazebo_simulation_generator/object_detect', ObjectSpawn, handle_yolo_detect_request)
	else:
		# Simple color recognition
		log('Starting object detecion with simple colour detecion')
		rospy.Subscriber("/find_object_node/object_pose", PoseStamped, simple_object_detect_listener)
		service = rospy.Service('gazebo_simulation_generator/object_detect', ObjectSpawn, handle_detect_request)
	
	rospy.sleep(5)

	log('starting service')


	rospy.spin()
