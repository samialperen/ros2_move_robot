cmake_minimum_required(VERSION 3.5)
project(ros2_draw_squares)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros2_draw_squares_custom_msgs REQUIRED)

# For user libraries
include_directories(include)

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

# Define our move_robot class as library target 
add_library(move_robot SHARED src/move_robot.cpp)
ament_target_dependencies(move_robot rclcpp std_msgs geometry_msgs)
ament_export_libraries(move_robot)

# Create an executable target for move_robot_node and link it with our library move_robot
add_executable(move_robot_node src/move_robot_node.cpp)
ament_target_dependencies(move_robot_node rclcpp std_msgs geometry_msgs)
target_link_libraries(move_robot_node move_robot)

# Define an executable target called move_robot_service_server and link it with our class
add_executable(move_robot_service_server src/move_robot_in_square_service_server.cpp)
ament_target_dependencies(move_robot_service_server rclcpp std_msgs geometry_msgs ros2_draw_squares_custom_msgs)
target_link_libraries(move_robot_service_server move_robot)

# Client just calls the service, there is no need to link it with move_robot class
add_executable(move_robot_service_client src/move_robot_in_square_service_client.cpp)
ament_target_dependencies(move_robot_service_client rclcpp std_msgs ros2_draw_squares_custom_msgs)

install(TARGETS
   move_robot
   move_robot_node
   move_robot_service_server
   move_robot_service_client
   DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY 
	include/
	DESTINATION include
)

# Install launch files.
install(DIRECTORY
 launch
 DESTINATION share/${PROJECT_NAME}/
)

ament_package()

