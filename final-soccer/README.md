# Robotics Final - Soccer

### To use the custom world file:

- Go to `ws_moveit/src/universal_robot/ur_gazebo/launch/inc/ur_control.launch.xml`
- (~Line 25), change `<arg name="gazebo_world" default="/home/jz327/finalproject/models/ur5_new_soccer_world.world" doc="The '.world' file to load in Gazebo." />` to the default path of your worldfile


### To launch the world:

- `roslaunch ur_gazebo ur5e_bringup.launch`
- (in another terminal) `roslaunch ur5e_moveit_config moveit_planning_execution.launch sim:=true`