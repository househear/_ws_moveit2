cmake_minimum_required(VERSION 3.5)
project(db_test1)

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


find_package(moveit_common REQUIRED)
moveit_package()
find_package(warehouse_ros_mongo REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(geometry_msgs REQUIRED)
find_package(rock_rhino_msgs REQUIRED)





# find dependencies
find_package(ament_cmake REQUIRED)


find_package(action_tutorials_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(include)

add_library(action_tutorials SHARED
  src/fibonacci_action_client.cpp
  src/db_action_server.cpp)
rclcpp_components_register_node(action_tutorials PLUGIN "db_test::FibonacciActionClient" EXECUTABLE fibonacci_action_client)
rclcpp_components_register_node(action_tutorials PLUGIN "db_test::ProCtrl2DataBaseActionServer" EXECUTABLE db_action_server)
target_compile_definitions(action_tutorials
  PRIVATE "DB_TEST_BUILDING_DLL")
ament_target_dependencies(action_tutorials
  "moveit_ros_planning_interface"
  "Boost"
  "warehouse_ros_mongo"
  "geometry_msgs"
  "rock_rhino_msgs"
  "action_tutorials_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")

install(TARGETS
  action_tutorials
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
