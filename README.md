# BWI_Multi_Robot
RUN commands

roslaunch bwi_launch multi_robot_simulation.launch
rosrun bwi_tasks multi_robot_navigation 

Main files:
bwi_common/bwi_tasks/src/multi_robot_navigation.cpp
bwi_common/utexas_gdc/maps/simulation/multimap2/3ne/objects.yaml

Defining objects (parking zones) on the map:
rosrun bwi_planning_common logical_marker _map_file:=/catkin_ws/src/bwi_common/utexas_gdc/maps/simulation/multimap2/3ne/3ne.yaml _data_directory:=/catkin_ws/src/bwi_common/utexas_gdc/maps/simulation/multimap2/3ne

Fetching the odometry readings of the two robots:
rostopic - /marvin/odom, /roberto/odom

Determining the interaction points:
rosservice – roberto/move_base/NavfnROS/make_plan (given roberto’s and marvin’s odom, returns a set of poses)
Check if collision occurs:
	Robots interaction point is in narrow area
	Robots moving towards each other
	
If collision detected:
	Determine parking zone closes to the interaction point (getNearestSafeZone)
		Parking zones defined in objects.yaml file (ia_)
	Find the robot thats closest to the parking zone - robot1
	Redirect robot1 to go to parking zone
	After robot2 passes robot1, resume robot1’s previous goal
