cmake_minimum_required(VERSION 3.5)
project(scan_fake)

set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/scan_fake/ScanFakePublisher.cpp
  src/scan_fake/ScanFakeSubscriber.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
add_executable(proceso
  src/proceso.cpp
)


target_link_libraries(proceso ${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${PROJECT_NAME}
  proceso
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
