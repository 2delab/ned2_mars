cmake_minimum_required(VERSION 3.22)
project(niryo_ned2_dual_arm_moveit_config)

find_package(ament_cmake REQUIRED)

# Install Python executable for joint state prefixer
# This node bridges unprefixed driver joint states to prefixed names expected by the URDF
install(
  PROGRAMS src/joint_state_prefixer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install ros2_controllers configuration
# This is loaded by the controller_manager node for hardware control
install(FILES config/ros2_controllers.yaml DESTINATION share/${PROJECT_NAME}/config)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
