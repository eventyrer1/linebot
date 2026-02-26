add_library(linebot_behaviors
  rectangle.cpp
)

ament_target_dependencies(linebot_behaviors geometry_msgs)

target_include_directories(linebot_behaviors PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
  $<INSTALL_INTERFACE:include>
)
