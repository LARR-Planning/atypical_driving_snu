cmake_minimum_required(VERSION 3.1.0)
project(atypical_driving)

set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})


find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

find_package(catkin REQUIRED roscpp sensor_msgs tf
            geometry_msgs octomap_msgs message_generation nav_msgs
            )

include_directories(include ${catkin_INCLUDE_DIRS})

file(GLOB SNU_PLANNER_SRC src/*.cpp src/Prediction/*.cpp)

catkin_package(
        LIBRARIES atypical_driving
        CATKIN_DEPENDS roscpp
        #  DEPENDS system_lib
)

add_executable(atypical_driving_test test/main.cpp ${SNU_PLANNER_SRC})
target_link_libraries(atypical_driving_test ${catkin_LIBRARIES})

add_executable(virtual_environment_generator test/virtual_environment_generator.cpp ${SNU_PLANNER_SRC})
target_link_libraries(virtual_environment_generator ${catkin_LIBRARIES})

add_executable(lane_tester test/LaneTest/main.cpp ${SNU_PLANNER_SRC})
target_link_libraries(lane_tester ${catkin_LIBRARIES})

add_executable(integration_server test/IntegrationTest/server_main.cpp ${SNU_PLANNER_SRC})
target_link_libraries(integration_server ${catkin_LIBRARIES})


add_executable(integration_client test/IntegrationTest/client_main.cpp ${SNU_PLANNER_SRC})
target_link_libraries(integration_client ${catkin_LIBRARIES})
