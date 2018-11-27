cmake_minimum_required(VERSION 2.8.3)
project(deepracer_interpreter)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  deepracer_msgs
)

catkin_package()

find_package(gazebo REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(racecar_plugin SHARED src/racecar_plugin.cc)
target_link_libraries(racecar_plugin ${GAZEBO_LIBRARIES})

install(TARGETS racecar_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
