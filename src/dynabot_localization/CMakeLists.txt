cmake_minimum_required(VERSION 3.8)
project(dynabot_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# install the imu_republisher.py script
install(PROGRAMS
  ${PROJECT_SOURCE_DIR}/dynabot_localization/imu_republisher.py
  DESTINATION lib/${PROJECT_NAME}
)
# install the include directory
install(
  DIRECTORY include
  DESTINATION include
)
# install the launch and config directories
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
