cmake_minimum_required(VERSION 3.5)
project(blackboard)

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(blackboard_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(rclcpp REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  blackboard_msgs
  geometry_msgs
  octomap_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/blackboard/BlackBoard.cpp
  src/blackboard/BlackBoardNode.cpp
  src/blackboard/BlackBoardClient.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(blackboard_main
  src/blackboard_main.cpp
)
ament_target_dependencies(blackboard_main ${dependencies})
target_link_libraries(blackboard_main ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  blackboard_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)

endif()

ament_package()
