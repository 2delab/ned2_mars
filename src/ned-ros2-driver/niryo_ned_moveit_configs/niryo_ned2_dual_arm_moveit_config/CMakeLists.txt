cmake_minimum_required(VERSION 3.22)
project(niryo_ned2_dual_arm_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python executables
# 1. Joint state prefixer - bridges unprefixed driver joint states to prefixed names (real hardware)
install(
  PROGRAMS src/joint_state_prefixer.py
  DESTINATION lib/${PROJECT_NAME}
)

# 2. Async executor node - asynchronous trajectory execution for dual-arm system (simulation + hardware)
install(
  PROGRAMS src/async_executor_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules from src/
# This makes the async executor modules importable by other nodes
ament_python_install_package(src)

# Install ros2_controllers configuration
# This is loaded by the controller_manager node for hardware control
install(FILES config/ros2_controllers.yaml DESTINATION share/${PROJECT_NAME}/config)

# Install async executor configuration
install(FILES config/async_executor.yaml DESTINATION share/${PROJECT_NAME}/config)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
