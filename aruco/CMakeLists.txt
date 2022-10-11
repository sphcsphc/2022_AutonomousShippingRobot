cmake_minimum_required(VERSION 3.0.2)
project(aruco)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
)

find_package(OpenCV 3.2.0 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
	geometry_msgs
	rospy
	sensor_msgs
	std_msgs
)

