cmake_minimum_required(VERSION 3.0.2)
project(point_cloud_to_grid_map)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
    std_msgs
    sensor_msgs
    pcl_ros
    pcl_conversions
    cv_bridge
    tf
    image_transport
    nav_msgs
    grid_map_ros
    grid_map_cv
    grid_map_msgs
)

catkin_package(
	# CATKIN_DEPENDS 
	# roscpp
    # std_msgs
    # sensor_msgs
    # pcl_ros
    # pcl_conversions
    # cv_bridge
    # tf
    # image_transport
    # nav_msgs
    # grid_map_ros
    # grid_map_cv
    # grid_map_msgs
)

find_package(OpenCV REQUIRED)

include_directories(
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	include
    )
    
link_directories(${PCL_LIBRARY_DIRS})

add_executable(point_cloud_to_grid_map
        src/point_cloud_to_grid_map.cpp
        )

target_include_directories(point_cloud_to_grid_map PRIVATE
        /include)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    # OpenCV
    ${OpenCV_LIBRARIES}
    # OpenCL
    ${PCL_LIBRARIES}
)        

add_dependencies(${PROJECT_NAME} 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)
