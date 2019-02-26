cmake_minimum_required(VERSION 2.8.3)
project(wall_follower)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(wall_follower src/wall_follower.cpp)

target_link_libraries(wall_follower ${catkin_LIBRARIES})