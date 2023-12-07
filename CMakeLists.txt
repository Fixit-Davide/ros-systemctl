cmake_minimum_required(VERSION 3.5)
project(systemctl_node)

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
find_package(PkgConfig REQUIRED)
pkg_check_modules(Systemd IMPORTED_TARGET GLOBAL libsystemd>=236)
if(NOT TARGET PkgConfig::Systemd)
    message(FATAL_ERROR "libsystemd of version at least 236 is required, but was not found "
                        "(if you have systemd in your OS, you may want to install package containing pkgconfig "
                        " files for libsystemd library. On Ubuntu, that is libsystemd-dev. "
                        " Alternatively, you may turn BUILD_LIBSYSTEMD on for sdbus-c++ to download, build "
                        "and incorporate libsystemd as embedded library within sdbus-c++)")
endif()
add_library(Systemd::Libsystemd ALIAS PkgConfig::Systemd)
string(REGEX MATCHALL "([0-9]+)" SYSTEMD_VERSION_LIST "${Systemd_VERSION}")
list(GET SYSTEMD_VERSION_LIST 0 LIBSYSTEMD_VERSION)
message(STATUS "Building with libsystemd v${LIBSYSTEMD_VERSION}")
include_directories(include)

add_library(${PROJECT_NAME} SHARED src/systemctl_node.cpp)
ament_target_dependencies(${PROJECT_NAME} SYSTEM rclcpp rcutils rclcpp_components std_srvs)
target_link_libraries(${PROJECT_NAME} Systemd::Libsystemd)

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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
