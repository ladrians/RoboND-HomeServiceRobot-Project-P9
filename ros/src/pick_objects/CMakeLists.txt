cmake_minimum_required(VERSION 2.8.3)
project(pick_objects)

find_package(catkin REQUIRED COMPONENTS
  #roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  actionlib
  move_base_msgs
)
catkin_package()

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(pick_objects src/pick_objects.cpp)
#target_link_libraries(pick_objects ${catkin_LIBRARIES})

