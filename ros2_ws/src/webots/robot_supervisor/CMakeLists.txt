cmake_minimum_required(VERSION 3.8)
project(robot_supervisor)

set(ENV{WEBOTS_HOME} "/usr/local/webots/")
set(CMAKE_EXPORT_COMPILE_COMMANDS YES)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

link_directories($ENV{WEBOTS_HOME}/lib/controller)
set(WEBOTS_LIBRARIES
    ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}
    ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX}
    ${CMAKE_SHARED_LIBRARY_PREFIX}CppDriver${CMAKE_SHARED_LIBRARY_SUFFIX}
    ${CMAKE_SHARED_LIBRARY_PREFIX}driver${CMAKE_SHARED_LIBRARY_SUFFIX})

include_directories($ENV{WEBOTS_HOME}/include/controller/c
                    $ENV{WEBOTS_HOME}/include/controller/cpp)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(webots_ros2_driver REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_node SHARED src/robot_supervisor.cpp)

ament_target_dependencies(${PROJECT_NAME}_node rclcpp rclcpp_components
                          image_transport std_msgs sensor_msgs)

target_link_libraries(${PROJECT_NAME}_node ${WEBOTS_LIBRARIES})

rclcpp_components_register_node(
  ${PROJECT_NAME}_node PLUGIN "example::RobotSupervisor" EXECUTABLE
  robot_supervisor)

# ========== Install Targets and Files ==========
install(
  TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_node)

ament_package()
