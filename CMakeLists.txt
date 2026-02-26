cmake_minimum_required(VERSION 3.8)
project(linebot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(linebot main.cpp)

ament_target_dependencies(linebot
  rclcpp
)

install(TARGETS linebot
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()