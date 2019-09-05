cmake_minimum_required(VERSION 3.5)
project(deepracer_interpreter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(deepracer_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)

include_directories(${rclcpp_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS}
                    ${deepracer_msgs_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(racecar_plugin SHARED src/racecar_plugin.cc)
target_link_libraries(
  racecar_plugin
  ${GAZEBO_LIBRARIES}
  ${deepracer_msgs_LIBRARIES}
)

install(TARGETS racecar_plugin
  LIBRARY DESTINATION lib
)

ament_export_dependencies(gazebo_ros_pkgs)
ament_export_dependencies(gazebo_ros)
ament_package()
