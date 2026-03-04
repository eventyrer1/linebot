cmake_minimum_required(VERSION 3.8)
project(linebot)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_subdirectory(src)

add_executable(linebot main.cpp)
ament_target_dependencies(linebot rclcpp geometry_msgs)
target_link_libraries(linebot linebot_behaviors)

install(TARGETS linebot
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()