cmake_minimum_required(VERSION 3.8)
project(coordinator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include_directories(include)

find_package(ament_cmake REQUIRED)

find_package(actions_cpp REQUIRED)
find_package(decorator_cpp REQUIRED)
find_package(control_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)

set(THIS_PACKAGE_DEPS actions_cpp decorator_cpp control_cpp behaviortree_cpp_v3
                      rclcpp nav2_behavior_tree)

add_executable(${PROJECT_NAME} src/coordinator.cpp)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_DEPS})

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY trees DESTINATION share/${PROJECT_NAME})

ament_package()
