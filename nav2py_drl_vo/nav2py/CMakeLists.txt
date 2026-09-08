cmake_minimum_required(VERSION 3.8)
project(nav2py)

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  nav2_core
  nav2_common
  geometry_msgs
)


install(DIRECTORY include/
DESTINATION include/
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

install(
  DIRECTORY nav2py
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_message_reader test/test_message_reader.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/nav2py)
endif()

ament_package(
  CONFIG_EXTRAS "nav2py-extras.cmake"
)
