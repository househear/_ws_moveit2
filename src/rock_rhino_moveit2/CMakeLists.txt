cmake_minimum_required(VERSION 3.5)
project(rock_rhino_moveit2)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
# This shouldn't be necessary (required by moveit_simple_controller_manager)
find_package(rosidl_default_runtime REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(run_ompl_constrained_planning src/run_ompl_constrained_planning.cpp)

add_executable(run_ompl_constrained_planning_while_run src/run_ompl_constrained_planning_while_run.cpp)
add_executable(process_controller src/process_controller.cpp)
add_executable(tcpip_test src/tcpip_test.cpp)
ament_target_dependencies(run_ompl_constrained_planning
  moveit_ros_planning_interface
  Boost
)

ament_target_dependencies(run_ompl_constrained_planning_while_run
  moveit_ros_planning_interface
  Boost
)

ament_target_dependencies(process_controller
  moveit_ros_planning_interface
  Boost
)


install(TARGETS run_ompl_constrained_planning
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)


install(TARGETS run_ompl_constrained_planning_while_run
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(TARGETS process_controller
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)


install(TARGETS tcpip_test
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
