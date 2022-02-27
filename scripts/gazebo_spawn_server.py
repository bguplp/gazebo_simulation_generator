#!/usr/bin/env python
# Omer Segal 25.9.20
# Email: omer91se@gmail.com
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from gazebo_simulation_generator.srv import ObjectSpawn

count = 0
OBJECT_MODEL_PATH = "/home/ronens-lab/catkin_ws/src/gazebo_worlds/Building_37/models/{}/model.sdf"

FLOOR_OBJECTS = ['small_table', 'chair']


def create_pose(x, y, z, oriented_x, oriented_y, oriented_z, oriented_w):
    initial_pose = Pose()
    initial_pose.position.x = x
    initial_pose.position.y = y
    initial_pose.position.z = z
    initial_pose.orientation.x = oriented_x
    initial_pose.orientation.y = oriented_y
    initial_pose.orientation.z = oriented_z
    initial_pose.orientation.w = oriented_w
    return initial_pose


def zero_z_for_floor_objects(object_name, z):
	if object_name in FLOOR_OBJECTS:
		return 0
	return z

# What: Insert an object to the current world with the given coordination
# object_name(string): the name of the object (e.g "table")
# x,y,z(int) are the pose position for the object.
# quaternion(geometry_msgs/Quaternion.msg): orientation of the object (None if not given)
def spawn_object(object_name, x, y, z, oriented_x, oriented_y, oriented_z, oriented_w):
	global count
	global OBJECT_MODEL_PATH

	use_gazebo = rospy.get_param('~use_gazebo', False)
	if use_gazebo:
		frame_id = 'armadillo2'
	else:
		frame_id = 'human_female_1_1'

	rospy.logdebug('[spawn_object]: Object name: %s.', object_name)
	rospy.logdebug('[spawn_object]: x: %s. y: %s. z: %s.', x, y, z)
	rospy.logdebug('[spawn_object]: orientation: x: %s. y: %s. z: %s. w: %s.',
			           oriented_x, oriented_y, oriented_z, oriented_w)

	z = zero_z_for_floor_objects(object_name, z)
	spawn_pose = create_pose(x, y, z, oriented_x, oriented_y, oriented_z, oriented_w)

	object_path = OBJECT_MODEL_PATH.format(object_name)
	try: 
		f = open(object_path, 'r')
		sdf = f.read()
	
		spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
		rospy.wait_for_service('gazebo/spawn_sdf_model')

		if object_name != 'human_female_1_1':
			spawned_object_name = object_name + str(count)
		else:
			frame_id = 'world'
			spawned_object_name = object_name

		res = spawn_model_prox(spawned_object_name, sdf, "robots_name_space", spawn_pose, frame_id)
		if res.success:
			count += 1
			return([True, spawned_object_name, 0, 0, 0, 0, 0, 0, 0])
			log('result: {}'.format(res))
		else:
			log('Not creating object')
			return ([False, spawned_object_name, 0, 0, 0, 0, 0, 0, 0])

	except Exception as e:
		log(e)
	finally:
		f.close()
		return ([False, spawned_object_name, 0, 0, 0, 0, 0, 0, 0])

def handle_spawn_object(req):
    log("Returning [%s]" % req.objectName)
    return spawn_object(req.objectName, req.x, req.y, req.z, req.oriented_x, req.oriented_y, req.oriented_z, 0)



def spawn_object_server():
	rospy.init_node('spawn_object_server', log_level=rospy.INFO)
	log('Ready to spawn objects.')

	rospy.Service('spawn_object', ObjectSpawn, handle_spawn_object)
	rospy.spin()


def log(msg, level='info'):
    if level is 'info':
        rospy.loginfo('[gazebo_spawn_server] ' + msg)


if __name__ == "__main__":
  spawn_object_server()


