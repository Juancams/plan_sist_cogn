cmake_minimum_required(VERSION 3.5)
project(house_nav)

set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_domain_expert REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    plansys2_msgs
    plansys2_executor
    plansys2_domain_expert
    plansys2_problem_expert
)

include_directories(include)

install(DIRECTORY include/
  DESTINATION include/
)

add_library(${PROJECT_NAME} SHARED 
  src/house_nav/Pick.cpp
  src/house_nav/Place.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(pick_action_node src/pick_action_node.cpp)
add_executable(place_action_node src/place_action_node.cpp)

target_link_libraries(pick_action_node ${PROJECT_NAME})
target_link_libraries(place_action_node ${PROJECT_NAME})

install(TARGETS
  pick_action_node
  place_action_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch pddl rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()