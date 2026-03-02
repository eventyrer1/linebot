cmake_minimum_required(VERSION 3.8)
project(linebot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
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

# Include headers
target_include_directories(linebot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Optional: build vendored PX4 optical flow plugin with its own upstream CMake
# when present in this repo.
set(PX4_OPTICAL_FLOW_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/px4_optical_flow/optical_flow")
if(EXISTS "${PX4_OPTICAL_FLOW_DIR}/CMakeLists.txt")
  add_subdirectory("${PX4_OPTICAL_FLOW_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/px4_optical_flow")
elseif(EXISTS "${PX4_OPTICAL_FLOW_DIR}")
  message(WARNING "Found ${PX4_OPTICAL_FLOW_DIR} but no CMakeLists.txt. Skipping vendored plugin build.")
endif()

install(TARGETS linebot
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
