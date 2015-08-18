cmake_minimum_required(VERSION 2.8.3)
project(turtle_star)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  angles
  geometry_msgs
  message_generation
  rosconsole
  roscpp
  std_msgs
  turtlesim
)

## Setup catkin exports
catkin_package()

## Set include dirs
include_directories(${catkin_INCLUDE_DIRS})

## star_drawer executable
add_executable(star_drawer src/star_drawer.cpp)
target_link_libraries(star_drawer ${catkin_LIBRARIES})
add_dependencies(star_drawer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Mark executables for installation
install(TARGETS star_drawer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
