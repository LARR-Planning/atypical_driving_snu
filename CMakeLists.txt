cmake_minimum_required(VERSION 3.15)
project(atypical_driving)

set(CMAKE_CXX_STANDARD 11)


find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

find_package(catkin REQUIRED roscpp sensor_msgs tf geometry_msgs octomap_msgs message_generation nav_msgs gazebo_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

file(GLOB SNU_PLANNER_SRC src/*.cpp)

add_executable(atypical_driving test/main.cpp ${SNU_PLANNER_SRC})