cmake_minimum_required(VERSION 3.5)
project(rock_rhino_database)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(warehouse_ros_mongo REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
# This shouldn't be necessary (required by moveit_simple_controller_manager)
find_package(rosidl_default_runtime REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(geometry_msgs REQUIRED)
find_package(action_tutorials_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)
include_directories(include)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
#include_directories(${WAREHOUSE_ROS_MONGO_INCLUDE_DIR})
#add_executable(run_ompl_constrained_planning src/run_ompl_constrained_planning.cpp)

add_library(test_demo SHARED
  src/test_demo.cpp
  )

rclcpp_components_register_node(test_demo PLUGIN "test_demo::DB_Manager" EXECUTABLE db_manager)
target_compile_definitions(test_demo
  PRIVATE "ROCK_RHINO_DATABASE_BUILDING_DLL")

  ament_target_dependencies(test_demo
  moveit_ros_planning_interface
  Boost
  warehouse_ros_mongo
  geometry_msgs
  action_tutorials_interfaces
  rclcpp_components
)



install(TARGETS test_demo
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)


ament_export_targets(export_${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
