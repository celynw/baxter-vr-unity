cmake_minimum_required(VERSION 2.8.3)
project(robotic_telepresence)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Find catkin macros and libraries
# If COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz) is used,
# also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	pcl_ros
)

## catkin specific configuration ###############################################
catkin_package()

## Build #######################################################################
# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
	${PROJECT_SOURCE_DIR}/include
	${catkin_INCLUDE_DIRS}
)

add_executable(kinect_sampler src/kinect_sampler.cpp)

target_link_libraries(kinect_sampler ${catkin_LIBRARIES})
