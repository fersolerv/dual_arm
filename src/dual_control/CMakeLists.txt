cmake_minimum_required(VERSION 3.0.2)
project(dual_control)

find_package(catkin REQUIRED COMPONENTS 
  rospy
  roslaunch
  gazebo_msgs
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
)

catkin_package()

roslaunch_add_file_check(launch)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)