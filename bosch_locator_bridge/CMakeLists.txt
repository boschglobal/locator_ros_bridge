cmake_minimum_required(VERSION 3.5)
project(bosch_locator_bridge)

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

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(rosidl_default_generators REQUIRED)

find_package(Poco REQUIRED COMPONENTS Foundation Net JSON)

set(msg_files
    "msg/ClientControlMode.msg"
    "msg/ClientGlobalAlignLandmarkObservationNotice.msg"
    "msg/ClientGlobalAlignLandmarkVisualizationInformation.msg"
    "msg/ClientGlobalAlignVisualization.msg"
    "msg/ClientLocalizationPose.msg"
    "msg/ClientLocalizationVisualization.msg"
    "msg/ClientMapVisualization.msg"
    "msg/ClientRecordingVisualization.msg"
)

set(srv_files
    "srv/ClientConfigGetEntry.srv"
    "srv/ClientMapList.srv"
    "srv/ClientMapSend.srv"
    "srv/ClientMapSet.srv"
    "srv/ClientMapStart.srv"
    "srv/StartRecording.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs std_srvs
)

ament_export_dependencies(rosidl_default_runtime)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/locator_bridge_node.cpp
  src/sending_interface.cpp
  src/receiving_interface.cpp
  src/rosmsgs_datagram_converter.cpp
  src/locator_rpc_interface.cpp)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME}_node  ${Boost_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_srvs
  tf2_geometry_msgs
  tf2_ros
  pcl_conversions
  Poco
)

rosidl_target_interfaces(${PROJECT_NAME}_node ${PROJECT_NAME} "rosidl_typesupport_cpp")


install(TARGETS ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
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
