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
ament_target_dependencies(linebot PUBLIC rclcpp geometry_msgs)
target_link_libraries(linebot PRIVATE linebot_behaviors)

target_include_directories(linebot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

foreach(dir launch models worlds)
  install(DIRECTORY ${dir}
    DESTINATION share/${PROJECT_NAME}
  )
endforeach()

ament_package()