cmake_minimum_required(VERSION 3.8)
project(nav2py_drl_vo_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2py REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(ament_index_cpp REQUIRED)

find_package(ament_cmake_venv REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# The policy runtime inherits a platform-provided PyTorch installation. This is
# important on Jetson, where NVIDIA's wheel must be used instead of a PyPI x86
# build. The remaining small, architecture-independent dependencies stay in the
# package-local virtual environment.
add_custom_command(
  OUTPUT "${venv_dir}/.drl_vo_runtime_ready"
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/requirements-runtime.txt"
  COMMAND "${Python3_EXECUTABLE}" -m venv --system-site-packages "${venv_dir}"
  COMMAND "${venv_python}" -c "import torch; print('Using platform PyTorch', torch.__version__)"
  COMMAND "${venv_python}" -m pip install --disable-pip-version-check --no-deps
    -r "${CMAKE_CURRENT_SOURCE_DIR}/requirements-runtime.txt"
  COMMAND "${venv_python}" -c
    "import cloudpickle, gym, matplotlib, pandas, stable_baselines3, yaml; print('DRL-VO Python runtime dependencies are importable')"
  COMMAND "${CMAKE_COMMAND}" -E touch "${venv_dir}/.drl_vo_runtime_ready"
  VERBATIM
)
add_custom_target("${target_venv}" ALL
  DEPENDS "${venv_dir}/.drl_vo_runtime_ready")
venv_install()
nav2py_package(nav2py_drl_vo_controller)
venv_script("compose_drl_vo_profile" "-m nav2py_drl_vo_controller.compose_profile")

include_directories(
  include
)

set(dependencies
  rclcpp
  pluginlib
  tf2
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  nav2_costmap_2d
  nav2_core
  nav2_util
  nav2py
  ament_index_cpp
  sensor_msgs
  std_msgs
)

add_library(nav2py_drl_vo_controller SHARED
  src/drl_vo_controller.cpp
  src/scan_preprocessor.cpp)

# prevent pluginlib from using boost
target_compile_definitions(nav2py_drl_vo_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(nav2py_drl_vo_controller
  ${dependencies}
)

install(TARGETS nav2py_drl_vo_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY model
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scan_preprocessor
    test/test_scan_preprocessor.cpp
    src/scan_preprocessor.cpp)
  target_include_directories(test_scan_preprocessor PRIVATE include)
  ament_target_dependencies(test_scan_preprocessor sensor_msgs)

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_planner test/test_planner.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/nav2py_drl_vo_controller)
  ament_add_pytest_test(test_compose_profile test/test_compose_profile.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/nav2py_drl_vo_controller)
endif()


ament_export_include_directories(include)
ament_export_libraries(nav2py_drl_vo_controller)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core nav2py_drl_vo_controller.xml)

ament_package()
