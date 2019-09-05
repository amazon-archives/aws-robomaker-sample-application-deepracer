cmake_minimum_required(VERSION 3.5)
project(deepracer_simulation)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)

#############
## Install ##
#############

install(PROGRAMS
    scripts/servo_commands.py
    scripts/run_local_rl_agent.sh
    scripts/run_rollout_rl_agent.sh
    scripts/run_evaluation_rl_agent.sh
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config models worlds
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Test
################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Macro for ament package
################################################################################
ament_export_dependencies(gazebo_ros_pkgs)
ament_package()
