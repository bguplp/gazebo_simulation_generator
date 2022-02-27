preliminaries:
1. in scripts/gazebo_spawn_server OBJECT_MODEL_PATH should have the correct path to the gazebo models.

Running:
 1. roslaunch armadillo2 armadillo2.launch gazebo:=true kinect:=true world_name:="`rospack find armadillo2_gazebo`/worlds/objects_on_table.world"
 2. roslaunch object_detection find_objects.launch
 3. rosrun gazebo_simulation_generator gazebo_spawn_server.py
 4. rosrun gazebo_simulation_generator object_detect.py
 5. rosrun gazebo_simulation_generator gazebo_spawn_client.py

