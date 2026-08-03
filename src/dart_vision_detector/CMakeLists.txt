cmake_minimum_required(VERSION 3.8)
project(dart_vision_detector)

# cmake-format: off
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
# cmake-format: on
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(dart_vision_interfaces REQUIRED)

find_package(OpenCV REQUIRED)

add_library(
  detector_core
  src/angle_filter.cpp
  src/light_detector.cpp
  src/pnp_solver.cpp
)

target_include_directories(
  detector_core
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
         ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
  detector_core
  PUBLIC ${OpenCV_LIBS}
)

add_executable(
  detector_node
  src/detector_main.cpp src/detector_node.cpp
)

target_include_directories(
  detector_node
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
  detector_node
  detector_core
)

ament_target_dependencies(
  detector_node
  rclcpp
  rcl_interfaces
  sensor_msgs
  std_msgs
  cv_bridge
  dart_vision_interfaces
)

install(
  TARGETS detector_core detector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the line when a
  # copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line when this package
  # is in a git repo and when a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
