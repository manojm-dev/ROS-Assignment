cmake_minimum_required(VERSION 3.8)
project(butlerbot_task)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
  scripts/task.py
  scripts/manual.py
  scripts/give_goal.py
  DESTINATION lib/${PROJECT_NAME}  # Install location
)

# Install other files like launch files, URDF, etc.
install(
  DIRECTORY 
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
