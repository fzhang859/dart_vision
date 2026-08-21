cmake_minimum_required(VERSION 3.8)
project(dart_vision_lidar_model)

if(CMAKE_COMPILER_IS_GNUCXX
   OR CMAKE_CXX_COMPILER_ID
      MATCHES
      "Clang"
)
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(
  Eigen3
  REQUIRED
  NO_MODULE
)
find_package(
  PCL
  QUIET
  REQUIRED
  COMPONENTS common
             filters
             io
)
find_package(yaml-cpp REQUIRED)

add_library(
  dart_vision_model_sampler SHARED
  src/model_sampler.cpp
)
target_include_directories(
  dart_vision_model_sampler
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)
target_include_directories(
  dart_vision_model_sampler SYSTEM
  PUBLIC ${PCL_INCLUDE_DIRS}
)
target_link_libraries(
  dart_vision_model_sampler
  PUBLIC Eigen3::Eigen ${PCL_LIBRARIES}
)
target_compile_definitions(
  dart_vision_model_sampler
  PUBLIC ${PCL_DEFINITIONS}
)

add_library(
  dart_vision_model_builder_library SHARED
  src/model_builder.cpp
)
target_include_directories(
  dart_vision_model_builder_library
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  dart_vision_model_builder_library
  PUBLIC dart_vision_model_sampler yaml-cpp
)

add_executable(
  dart_vision_model_builder
  src/model_builder_cli.cpp
)
target_link_libraries(
  dart_vision_model_builder
  PRIVATE dart_vision_model_builder_library
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models/processed DESTINATION share/${PROJECT_NAME}/models)
install(
  TARGETS dart_vision_model_sampler
          dart_vision_model_builder_library
          dart_vision_model_builder
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3
  PCL
  yaml-cpp
)

ament_package()
