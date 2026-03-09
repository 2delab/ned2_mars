cmake_minimum_required(VERSION 3.22)
project(niryo_ned2_nrobot_moveit_config)

find_package(ament_cmake REQUIRED)

# Custom target to generate configs from robot_config.yaml before installation
add_custom_target(generate_configs ALL
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_all.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Generating MoveIt configs from robot_config.yaml"
)

# Install Python executable for joint state prefixer
install(
  PROGRAMS src/joint_state_prefixer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config directory (includes both source and generated files)
install(DIRECTORY config 
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE
)

# Install scripts for user reference
install(
  DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
