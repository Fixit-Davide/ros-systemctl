cmake_minimum_required(VERSION 3.5)
project(systemctl_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_srvs REQUIRED)
find_package(libsystemd REQUIRED)
include_directories(include)

add_library(${PROJECT_NAME} SHARED src/systemctl_node.cpp)
ament_target_dependencies(${PROJECT_NAME} SYSTEM rclcpp rcutils rclcpp_components std_srvs libsystemd)

rclcpp_components_register_node(
  ${PROJECT_NAME} 
  PLUGIN "addons::SystemctlController"
  EXECUTABLE SystemctlControllerExecutable)

install(TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
