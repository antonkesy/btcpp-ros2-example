cmake_minimum_required(VERSION 3.16)
project(action_interfaces)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(actions Move Print Runtime IsFrontClear)

foreach(action IN LISTS actions)
  list(APPEND interfaces "action/${action}.action")
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME} "${interfaces}")

ament_export_dependencies(rosidl_default_runtime)
ament_package()
