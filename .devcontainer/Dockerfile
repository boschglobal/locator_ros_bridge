FROM osrf/ros:humble-desktop

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
     && apt-get -y install --no-install-recommends \
        ros-humble-nav2-util\
        ros-humble-ros-gz \
        ros-humble-turtlebot3-gazebo\
        ros-humble-turtlebot3-teleop\
        libpoco-dev\
        mosquitto mosquitto-clients
        
 

ENV TURTLEBOT3_MODEL=burger
