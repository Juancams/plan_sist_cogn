cmake_minimum_required(VERSION 3.5)
project(cognitive_arch)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_problem_expert REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  rclcpp_action
  geometry_msgs
  plansys2_msgs
  nav2_msgs
  rclcpp_cascade_lifecycle
  lifecycle_msgs
  plansys2_executor
  plansys2_problem_expert
  plansys2_domain_expert
)

include_directories(fsm include)

add_library(${PROJECT_NAME} SHARED
  src/cognitive_arch/Explore.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(hfsm_main
  src/ca_HFSMImpl.cpp
  fsm/ca_HFSM.cpp
)

add_executable(explore_action src/explore_action.cpp)

target_link_libraries(explore_action ${PROJECT_NAME})

install(DIRECTORY launch pddl rviz DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)
ament_target_dependencies(hfsm_main ${dependencies})

install(TARGETS
  ${PROJECT_NAME}
  hfsm_main

  explore_action
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
