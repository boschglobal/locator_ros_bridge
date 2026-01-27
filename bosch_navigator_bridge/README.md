General information about this repository, including legal information and build instructions are given in [README.md](../README.md) in the repository root.

# bosch_navigator_bridge

## Overview

This package provides a [ROS 2] interface to the [Rexroth ROKIT Navigator].
It translates ROS2 messages to the ROKIT Navigation API (as describe in the ROKIT Navigator API documentation) and vice versa.
It is important to know that the bridge can only function when using the Locator.

The package has been tested under [ROS 2] Humble and Ubuntu 22.04.
The bridge is compatible with ROKIT Navigator version 2.0.


The following video gives more information about the ROKIT Navigator:
[[Rexroth ROKIT Navigator]](https://www.youtube.com/watch?v=7DCcJIb_GD8)

## Quick Start
This shows you how to start the bridge.

#### Ensure the ROKIT Navigator is reachable from your computer

Make sure the ROKIT Navigator is installed and running on a computer in your network. You can test this by running the following command in a terminal (replace `<NAVIGATOR_IP>` by the IP address of the computer running the ROKIT Navigator):

```sh
curl --header "Content-Type: application/json" --request POST --data '{"jsonrpc":"2.0","method":"aboutModulesList","params":{"query":{}},"id":1}' http://<NAVIGATOR_IP>:8082
```

#### Start Bridge Node

Start the bridge node with

    ros2 launch bosch_navigator_bridge bridge.launch.xml bridge_ip:=<HOST_IP> nav_ip:=<NAVIGATOR_IP> loc_ip:=<LOCATOR_IP> nav_user:=<USER> nav_password:=<PASSWORD> odometry_pose_set:=<ODOMETRY_POSE_SET> odom_topic:=<ODOM_TOPIC> cmd_vel_topic:=<TWIST_TOPIC> feedback_datagram_port:=<FEEDBACK_PORT>

 where
- `<HOST_IP>` is the IP address of the computer the bridge is to be started
- `<NAVIGATOR_IP>` is the IP address of the computer where the ROKIT Navigator is running
- `<LOCATOR_IP>` is the IP address of the computer where the ROKIT Locator is running 
- `<USER>` and `<PASSWORD>` are the credentials to log into the ROKIT Navigator
- `<ODOMETRY_POSE_SET>` this determines whether the pose within the odometry is considered valid and used or only the twist
- `<ODOM_TOPIC>` is the topic name of the odometry 
- `<TWIST_TOPIC>` is the topic name of the twist
- `<FEEDBACK_PORT>` ist the port of the client motion feedback interface

Since ROKIT Navigator is running inside a container, the `<HOST_IP>` has to be set to docker host IP address (e.g 172.17.0.1) instead of localhost IP address (127.0.0.1).

For additional arguments please refer to the launch file [bridge.launch.xml](./launch/bridge.launch.xml).

For the bridge to function completely, the navigator's automatic mode must be started via the aXessor. Additionally, the vehicle kinematics and limits must be configured through the aXessor. The connection to the FMS's MQTT broker must also be configured there.

## Nodes

### bridge_node

This node provides an interface to the navigation client.

#### ROKIT Navigator Configuration

For a correct configuration, it is important that `ClientMotion.feedback.address` is set to the IP address (with port) of the computer where the Navigator Bridge is running. Additionally, it is important that `ClientLocalization.pose.address` is set to IP address (with port) from which the Navigator container can reach the Locator.  


#### Subscribed Topics

* **`/odom`** ([nav_msgs/msg/Odometry])

    The Odom topic is used for feedback, which is then forwarded to the ROKIT Navigator. 


#### Published Topics

* **`/cmd_vel`** ([geometry_msgs/msg/Twist])
  
    The Twist topic is the command topic for the Client Motion. Command inputs are converted to Twist messages and published on the `cmd_vel`topic.

#### Parameters

* **`nav_host`**
     The IP address of the computer where ROKIT Navigator is running
* **`nav_rpc_port`**
     The port for the ROKIT Navigator JSON RPC client interface
* **`nav_binary_ports_start`**
     The port where the binary interface starts
* **`feedback_datagram_port`**
     The port for the client motion feedback interface, coming from the Bridge
* **`user_name`**
     The user name of the navigator
* **`password`**
     The password of the navigator
* **`odometry_pose_set`**
     This determines whether the pose within the odometry is considered valid and used or only the twist
