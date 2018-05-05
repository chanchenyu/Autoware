cmake_minimum_required(VERSION 2.8.12)
project(multi_lidar_fusion)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        geometry_msgs
        pcl_ros
        pcl_conversions
        )

catkin_package(CATKIN_DEPENDS
        std_msgs
        sensor_msgs
        geometry_msgs
        )

find_package(Qt5Core REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
        include
)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

link_directories(${PCL_LIBRARY_DIRS})

# MultiLidar Fusion
add_library(multi_lidar_fusion_lib SHARED
        src/multi_lidar_fusion.cpp
        include/multi_lidar_fusion.h)

target_include_directories(multi_lidar_fusion_lib PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        )

target_link_libraries(multi_lidar_fusion_lib
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Qt5Core_LIBRARIES}
        )

add_executable(multi_lidar_fusion
        src/multi_lidar_fusion_node.cpp
        )

target_include_directories(multi_lidar_fusion PRIVATE
        include)

target_link_libraries(multi_lidar_fusion
        multi_lidar_fusion_lib)