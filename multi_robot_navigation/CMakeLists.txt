cmake_minimum_required(VERSION 2.8.3)
project(multi_robot_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  bwi_msgs
  bwi_logical_translator
  nav_msgs
  plan_execution
  roscpp
  rospy
  std_msgs
  tf_conversions
  visualization_msgs
  multi_robot_collision_avoidance
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs bwi_msgs std_msgs plan_execution
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})
# class definition & visit door list embeded
add_executable(sample_safety_model src/sample_safety_model.cpp)
add_dependencies(sample_safety_model ${catkin_EXPORTED_TARGETS})
target_link_libraries(sample_safety_model ${catkin_LIBRARIES})
# Only chicken safety class is embeded so that others can use it by import this module
add_executable(chicken_safety_model src/chicken_safety_model.cpp src/ChickenSafety.cpp)
add_dependencies(chicken_safety_model ${catkin_EXPORTED_TARGETS})
target_link_libraries(chicken_safety_model ${catkin_LIBRARIES})

# Experiment controller
add_executable(exp_handler src/exp_handler.cpp src/ChickenSafety.cpp)
add_dependencies(exp_handler ${catkin_EXPORTED_TARGETS})
target_link_libraries(exp_handler ${catkin_LIBRARIES})

add_executable(exp_vanilla_navstack_orientation_test src/exp_vanilla_navstack_orientation_test.cpp)
add_dependencies(exp_vanilla_navstack_orientation_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(exp_vanilla_navstack_orientation_test ${catkin_LIBRARIES})

add_executable(exp_parking_spot_orientation_test src/exp_parking_spot_orientation_test.cpp)
add_dependencies(exp_parking_spot_orientation_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(exp_parking_spot_orientation_test ${catkin_LIBRARIES})

# Distributed version visit_door_list
add_executable(fixed_marvin src/fixed_marvin.cpp)
add_dependencies(fixed_marvin ${catkin_EXPORTED_TARGETS})
target_link_libraries(fixed_marvin ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

