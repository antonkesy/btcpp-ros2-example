cmake_minimum_required(VERSION 3.5)
project(actions_cpp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(action_interfaces REQUIRED)

set(dependencies
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    nav2_msgs
    behaviortree_cpp_v3
    nav2_behaviors
    action_interfaces)

set(plugin_names move print get_runtime is_front_clear)

foreach(plugin_name ${plugin_names})
  # Clients
  set(client_name ${plugin_name}_action)
  add_library(${client_name} SHARED src/${plugin_name}/${plugin_name}.cpp)
  ament_target_dependencies(${client_name} ${dependencies})
  target_compile_definitions(${client_name} PRIVATE BT_PLUGIN_EXPORT)
  list(APPEND clients ${client_name})

  # Server
  set(server_name ${plugin_name}_server)
  set(server_file src/${plugin_name}/${server_name}.cpp)
  add_executable(${server_name} ${server_file})
  ament_target_dependencies(${server_name} ${dependencies})
  list(APPEND servers ${server_name})
endforeach()

install(TARGETS ${servers} DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS ${clients}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/)

ament_export_libraries(${clients})
ament_export_dependencies(${dependencies})
ament_export_include_directories(include)

ament_package()
