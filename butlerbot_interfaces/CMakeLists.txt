cmake_minimum_required(VERSION 3.8)
project(butlerbot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate action and service files
rosidl_generate_interfaces(${PROJECT_NAME}
  action/FoodDelivery.action
  srv/Confirm.srv
)

# Ensure dependencies are listed
ament_export_dependencies(rosidl_default_runtime)

ament_package()
