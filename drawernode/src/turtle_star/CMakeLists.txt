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

## shape_server executable
add_executable(star_server src/star_server.cpp)
target_link_libraries(star_server ${catkin_LIBRARIES})
add_dependencies(star_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## shape_client executable
add_executable(star_client src/star_client.cpp)
target_link_libraries(star_client ${catkin_LIBRARIES})
add_dependencies(star_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Mark executables for installation
install(TARGETS star_server star_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
