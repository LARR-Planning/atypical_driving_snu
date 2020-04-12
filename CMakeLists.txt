cmake_minimum_required(VERSION 3.1.0)
project(atypical_driving)

set(CMAKE_CXX_STANDARD 11)


find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

find_package(catkin REQUIRED roscpp sensor_msgs tf
            geometry_msgs octomap_msgs message_generation nav_msgs
            )

include_directories(include ${catkin_INCLUDE_DIRS})

file(GLOB SNU_PLANNER_SRC src/*.cpp)

catkin_package(
        LIBRARIES atypical_driving
        CATKIN_DEPENDS roscpp
        #  DEPENDS system_lib
)

add_executable(atypical_driving_test test/main.cpp ${SNU_PLANNER_SRC})
target_link_libraries(atypical_driving_test ${catkin_LIBRARIES})
