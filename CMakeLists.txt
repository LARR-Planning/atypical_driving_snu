cmake_minimum_required(VERSION 3.1.0)
project(atypical_driving)

set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -O3")
#set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")


find_package(Eigen3  REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIRS})


find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs tf tf2
            geometry_msgs octomap_msgs message_generation nav_msgs message_generation driving_msgs pcl_ros
            )

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
file(GLOB SNU_PLANNER_SRC src/optimization_module/*.cpp src/*.cpp src/Prediction/*.cpp src/third_party/*.cpp )


catkin_package(
        LIBRARIES atypical_driving
        CATKIN_DEPENDS roscpp
        #  DEPENDS system_lib
)

add_executable(atypical_driving_test test/main.cpp ${SNU_PLANNER_SRC} src/optimization_module/math.cpp)
target_link_libraries(atypical_driving_test ${catkin_LIBRARIES} -lstdc++fs )


add_executable(virtual_environment_generator test/virtual_environment_generator.cpp ${SNU_PLANNER_SRC})
target_link_libraries(virtual_environment_generator ${catkin_LIBRARIES} -lstdc++fs)

add_executable(lane_tester test/LaneTest/main.cpp ${SNU_PLANNER_SRC})
target_link_libraries(lane_tester ${catkin_LIBRARIES} -lstdc++fs)

add_executable(integration_server test/IntegrationTest/server_main.cpp ${SNU_PLANNER_SRC})
target_link_libraries(integration_server ${catkin_LIBRARIES} -lstdc++fs)

add_executable(integration_client test/IntegrationTest/client_main.cpp ${SNU_PLANNER_SRC})
target_link_libraries(integration_client ${catkin_LIBRARIES} -lstdc++fs)

add_executable(keti_client test/KetiTest/client_main.cpp ${SNU_PLANNER_SRC})
target_link_libraries(keti_client ${catkin_LIBRARIES} ${PCL_LIBRARIES})
