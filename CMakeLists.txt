cmake_minimum_required(VERSION 3.0.2)
project(coverage_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)

catkin_python_setup()
