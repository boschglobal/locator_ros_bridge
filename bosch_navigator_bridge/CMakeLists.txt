cmake_minimum_required(VERSION 3.8)
project(bosch_navigator_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation Net JSON)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/navigator_bridge_node.cpp
  src/sending_interface.cpp
  src/receiving_interface.cpp
  src/rosmsgs_datagram_converter.cpp
  src/navigator_interface.cpp)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME  navigator_bridge_node PREFIX "")

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>${tf2_INCLUDE_DIRS}${tf2_geometry_msgs_INCLUDE_DIRS})

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  geometry_msgs
  nav_msgs
  Poco
  tf2
  tf2_geometry_msgs
)


# Install the executable
install(TARGETS ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
