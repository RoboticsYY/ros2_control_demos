cmake_minimum_required(VERSION 3.5)
project(ros2_control_demo_hardware)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# Find shmringbuf
set(LIBSHM_ROOT_DIR "/usr/local" CACHE PATH "Root directory of libshm.")
execute_process(COMMAND find ${LIBSHM_ROOT_DIR}/include -name "shmringbuf.h"
                COMMAND head -n 1
                COMMAND xargs dirname
                OUTPUT_VARIABLE LIBSHM_INCLUDE_DIRS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND find ${LIBSHM_ROOT_DIR}/lib -name "*libshmringbuf.so"
                COMMAND head -n 1
                OUTPUT_VARIABLE LIBSHM_LIBRARIES OUTPUT_STRIP_TRAILING_WHITESPACE)
message(STATUS "LIBSHM_INCLUDE_DIRS: ${LIBSHM_INCLUDE_DIRS}")
message(STATUS "LIBSHM_LIBRARIES: ${LIBSHM_LIBRARIES}")

include_directories(
  ${LIBSHM_INCLUDE_DIRS}
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

## COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  src/diffbot_system.cpp
  src/rrbot_system_position_only.cpp
  src/rrbot_system_multi_interface.cpp
  src/rrbot_system_with_sensor.cpp
  src/rrbot_system_igh.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
)

target_link_libraries(${PROJECT_NAME} ${LIBSHM_LIBRARIES})

pluginlib_export_plugin_description_file(hardware_interface ros2_control_demo_hardware.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)
ament_package()
