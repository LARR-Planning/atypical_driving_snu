cmake_minimum_required(VERSION 3.1.0)
project(atypical_driving)

set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -lpthread -std=c++11")
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

find_package(catkin REQUIRED roscpp sensor_msgs tf
            geometry_msgs octomap_msgs message_generation nav_msgs
            )

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
file(GLOB SNU_PLANNER_SRC src/optimization_module/*.cpp src/*.cpp )

catkin_package(
        LIBRARIES atypical_driving
        CATKIN_DEPENDS roscpp
        #  DEPENDS system_lib
)

add_executable(atypical_driving_test test/main.cpp ${SNU_PLANNER_SRC} src/optimization_module/math.cpp)
target_link_libraries(atypical_driving_test ${catkin_LIBRARIES} )

add_executable(virtual_environment_generator test/virtual_environment_generator.cpp ${SNU_PLANNER_SRC} src/optimization_module/math.cpp)
target_link_libraries(virtual_environment_generator ${catkin_LIBRARIES} )
