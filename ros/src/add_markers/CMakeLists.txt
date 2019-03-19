cmake_minimum_required(VERSION 2.8.3)
project(add_markers)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  visualization_msgs
)
catkin_package()

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
