#Run commands
- roslaunch bwi_launch multi_robot_simulation.launch
- rosrun bwi_tasks multi_robot_navigation 

#Main files
- bwi_common/bwi_tasks/src/multi_robot_navigation.cpp
- bwi_common/utexas_gdc/maps/simulation/multimap2/3ne/objects.yaml

#Utils
- Defining objects (parking zones) on the map:
--- rosrun bwi_planning_common logical_marker _map_file:=/home/users/NI2452/catkin_ws/src/bwi_common/utexas_gdc/maps/simulation/3ne/3ne.yaml _data_directory:=/home/users/NI2452/catkin_ws/src/bwi_common/utexas_gdc/maps/simulation/3ne

#Algorithm
- Fetching the odometry readings of the two robots:
--- rostopic - /marvin/odom, /roberto/odom

- Determining the interaction points:
--- rosservice – roberto/move_base/NavfnROS/make_plan (given roberto’s and marvin’s odom, returns a set of poses)

- Check if collision occurs:
	* Robots interaction point (ip) is in narrow area
	(If a parking zone lies inside the circle having ip as center and radius=2 grid cells, then interaction happens in safe zone)
	* Robots moving towards each other
- If collision detected

	**Determine parking zone closest to the interaction point (getNearestParkingZone)
		Parking zones defined in objects.yaml file (p3_)
		
	**Find the robot thats closest to the parking zone - robot1
	
	**Redirect robot1 to go to parking zone
	
	**After robot2 passes robot1, resume robot1’s previous goal
