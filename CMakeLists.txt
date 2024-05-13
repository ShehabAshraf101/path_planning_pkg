cmake_minimum_required(VERSION 3.0.2)
project(path_planning_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
    # Include any additional ROS message packages if needed
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
  FILES
  Waypoint.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS msg
  LIBRARIES vehicle_control_pkg
  CATKIN_DEPENDS rospy std_msgs geometry_msgs message_runtime# Update this with additional dependencies if needed
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable (if you have any other executables)
# add_executable(example_node src/example.cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(example_node
#   ${catkin_LIBRARIES}
# )

## Install executable (if you have any other executables)
# install(TARGETS example_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

#############
## Install ##
#############

# No additional installation required

#############
## Testing ##
#############

# No tests defined in this package

