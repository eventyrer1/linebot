file(GLOB LINEBOT_BEHAVIOR_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

add_library(linebot_behaviors ${LINEBOT_BEHAVIOR_SOURCES})

ament_target_dependencies(linebot_behaviors
  rclcpp
  geometry_msgs
)

target_include_directories(linebot_behaviors PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS linebot_behaviors
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)