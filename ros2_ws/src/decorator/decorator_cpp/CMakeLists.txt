cmake_minimum_required(VERSION 3.5)
project(decorator_cpp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

set(dependencies rclcpp behaviortree_cpp_v3)

set(plugin_names random_delay)

foreach(plugin_name ${plugin_names})
  set(client_name ${plugin_name}_decorator)
  add_library(${client_name} SHARED src/${plugin_name}/${plugin_name}.cpp)
  ament_target_dependencies(${client_name} ${dependencies})
  target_compile_definitions(${client_name} PRIVATE BT_PLUGIN_EXPORT)
  list(APPEND clients ${client_name})
endforeach()

install(
  TARGETS ${clients}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include/)

ament_export_libraries(${clients})
ament_export_dependencies(${dependencies})
ament_export_include_directories(include)

ament_package()
