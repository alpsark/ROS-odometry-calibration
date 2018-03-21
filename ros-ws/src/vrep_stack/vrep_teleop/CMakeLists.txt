cmake_minimum_required(VERSION 2.8.3)
project(vrep_teleop)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs joy)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS
#  LIBRARIES diff_joy_control
#  CATKIN_DEPENDS roscpp geometry_msgs joy
#  DEPENDS
)

###########
## Build ##
###########

add_executable(diff_joy_control src/diff_joy_control.cpp)
add_executable(diff_key_control src/diff_key_control.cpp)
add_executable(diff_key_publish src/diff_key_publish.cpp)
target_link_libraries(diff_joy_control ${catkin_LIBRARIES})
target_link_libraries(diff_key_control ${catkin_LIBRARIES})
target_link_libraries(diff_key_publish ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
#install(PROGRAMS
#  scripts/diff_key_control
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark executables and/or libraries for installation
#install(TARGETS diff_joy_control
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark all other useful stuff for installation
#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
