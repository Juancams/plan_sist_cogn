cmake_minimum_required(VERSION 3.5)
project(taller)

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


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(practica_msgs REQUIRED)


set(dependencies
    rclcpp
    rclcpp_lifecycle
    std_msgs
    geometry_msgs
    practica_msgs
)

include_directories(include)


add_executable(proceso_1 src/proceso_1.cpp)
ament_target_dependencies(proceso_1 ${dependencies})

add_executable(proceso_2 src/proceso_2.cpp)
ament_target_dependencies(proceso_2 ${dependencies})

add_executable(proceso_3 src/proceso_3.cpp)
ament_target_dependencies(proceso_3 ${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()


install(TARGETS
  proceso_1
  proceso_2
  proceso_3
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()