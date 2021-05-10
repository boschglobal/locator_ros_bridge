cmake_minimum_required(VERSION 2.8.3)
project(bosch_locator_bridge)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
        message_generation
        nav_msgs
        pcl_conversions
        roscpp
        sensor_msgs
        std_msgs
        std_srvs
        tf2
        tf2_geometry_msgs
        tf2_ros
)

find_package(Poco REQUIRED COMPONENTS Foundation Net JSON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# under melodic, we get a deprecation warning from a ROS include
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations")

add_message_files(
  DIRECTORY
  msg
  FILES
    ClientControlMode.msg
    ClientGlobalAlignLandmarkObservationNotice.msg
    ClientGlobalAlignLandmarkVisualizationInformation.msg
    ClientGlobalAlignVisualization.msg
    ClientLocalizationPose.msg
    ClientLocalizationVisualization.msg
    ClientMapVisualization.msg
    ClientRecordingVisualization.msg
)

add_service_files(
  FILES
    ClientMapList.srv
    ClientMapSend.srv
    ClientMapSet.srv
    ClientMapStart.srv
    StartRecording.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    nav_msgs
    pcl_conversions
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/locator_bridge_node.cpp
  src/sending_interface.cpp
  src/receiving_interface.cpp
  src/rosmsgs_datagram_converter.cpp
  src/locator_rpc_interface.cpp)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  Poco::Foundation
  Poco::JSON
  Poco::Net
)

install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES config/locator_bridge_localization.rviz config/locator_bridge_map_creation.rviz config/locator_bridge_visual_recording.rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(FILES launch/bridge.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
