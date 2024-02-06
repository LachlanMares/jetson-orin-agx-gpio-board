cmake_minimum_required(VERSION 3.8)
project(jetson_orin_agx_gpio_board)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GpioStates.msg"
  "srv/GpioSetOutput.srv"
  DEPENDENCIES # Add packages that above messages depend on
)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
# add_executable(cpp_node src/cpp_node.cpp)
# ament_target_dependencies(cpp_node rclcpp)    
                 
# Install Cpp executables
# install(TARGETS
#   cpp_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Export the launch files.
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  scripts/jetson_gpio_interface.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
