cmake_minimum_required(VERSION 3.14)
project(tram_odometry)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
option(TRAM_ODOMETRY_SANITIZE "Build with AddressSanitizer/UBSan (long-run leak/race checks)" OFF)
if(TRAM_ODOMETRY_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tram_vehicle_msgs REQUIRED)

# ROS-independent estimator core (also used by the offline replay tool)
add_library(tram_odometry_core STATIC
  src/core/json_lite.cpp
  src/core/geo.cpp
  src/core/route_map.cpp
  src/core/traction_model.cpp
  src/core/estimator.cpp)
target_include_directories(tram_odometry_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(tram_odometry_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(tram_odometry_node src/tram_odometry_node.cpp)
target_link_libraries(tram_odometry_node tram_odometry_core)
ament_target_dependencies(tram_odometry_node
  rclcpp ament_index_cpp nav_msgs sensor_msgs std_msgs diagnostic_msgs builtin_interfaces
  tram_vehicle_msgs)

add_executable(tram_odometry_replay src/replay_main.cpp)
target_link_libraries(tram_odometry_replay tram_odometry_core)

install(TARGETS tram_odometry_node tram_odometry_replay
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/result_recorder.py
  scripts/evaluate_run.py
  scripts/run_demo.sh
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(tram_odometry_core_test test/core_test.cpp)
  target_link_libraries(tram_odometry_core_test tram_odometry_core)
  add_test(NAME tram_odometry_core_test
    COMMAND tram_odometry_core_test ${CMAKE_CURRENT_SOURCE_DIR}/maps/route_closed.json)
endif()

ament_package()
