General information about this repository, including legal information and build instructions are given in [README.md](../README.md) in the repository root.

# bosch_locator_bridge

## Overview

This package provides a [ROS 1] interface to the [Rexroth ROKIT Locator].
It translates ROS 1 messages to the ROKIT Locator API (as described in the ROKIT Locator API documentation) and vice versa.
It also allows to control the ROKIT Locator via ROS 1 service calls.

The package has been tested under [ROS 1] Noetic and Ubuntu 20.04.
The bridge is compatible with ROKIT Locator version 1.6.
If you have an earlier version, see [Support of earlier versions of ROKIT Locator](#support-of-earlier-versions-of-rokit-locator).

## Quick Start

This section describes how to record your environment, create a map out of the record and localize yourself within it.

#### Ensure the ROKIT Locator is reachable from your computer

Make sure the ROKIT Locator is installed and running on a computer in your network. You can test this by running the following command in a terminal (replace `<LOCATOR_IP>` by the IP address of the computer running the ROKIT Locator):
```sh
curl --header "Content-Type: application/json" --request POST --data '{"jsonrpc":"2.0","method":"aboutModulesList","params":{"query":{}},"id":1}' http://<LOCATOR_IP>:8080
```

#### Start Bridge Node

Start the bridge node with

    roslaunch bosch_locator_bridge bridge.launch bridge_ip:=<HOST_IP> locator_ip:=<LOCATOR_IP> locator_user:=<USER> locator_password:=<PASSWORD> scan_topic:=<SCAN_TOPIC> enable_odometry:=<ENABLE_ODOM> odom_topic:=<ODOM_TOPIC>

where
- `<HOST_IP>` is the IP address of the computer the bridge is to be started
- `<LOCATOR_IP>` is the IP address of the computer where the ROKIT Locator is running
- `<USER>` and `<PASSWORD>` are the credentials to log into the ROKIT Locator
- `<SCAN_TOPIC>` is the topic name of the laser scans
- `<ENABLE_ODOM>` is a boolean that describes whether you want to forward odometry ROS messages to the ROKIT Locator
- `<ODOM_TOPIC>` is the topic name of the odometry

Since the Laser Localization Software is running in docker, the `<HOST_IP>` has to be set to docker0 ip address (172.17.0.1) instead of localhost ip address (127.0.0.1) when the user program providing the laser data runs on the same machine.

For additional parameters please refer to the launch file [bridge.launch](./launch/bridge.launch).

#### Start Visual Recording

Then, make sure there are laser scans published on `<SCAN_TOPIC>`, and start the visual recording with e.g.

    rosrun rviz rviz -d `rospack find bosch_locator_bridge`/config/locator_bridge_visual_recording.rviz
    rosservice call /bridge_node/start_visual_recording "name: 'ROS-Quickstart-$(date -Isecond)'"

You should see the laser scans, the robot position and its previous path, and the recording  built up over time.

#### Stop Visual Recording

When you are done, you can stop recording with

    rosservice call /bridge_node/stop_visual_recording

#### Create Map From Recording

Now, create a map from your recording with

    rosrun rviz rviz -d `rospack find bosch_locator_bridge`/config/locator_bridge_map_creation.rviz
    rosservice call /bridge_node/start_map "" ""

When the map has been created, it must be sent to the map server and set as the active map.

    rosservice call /bridge_node/send_map ""
    rosservice call /bridge_node/set_map ""

#### Start Localization

Finally, you can start the localization with

    rosrun rviz rviz -d `rospack find bosch_locator_bridge`/config/locator_bridge_localization.rviz
    rosservice call /bridge_node/start_localization

You can set the initial pose of the robot by clicking the "2D Pose Estimate" button in RViz, and then set the pose in the map.

#### Stop Localization

When you are done, you can stop the localization with

    rosservice call /bridge_node/stop_localization

## Nodes

### bridge_node

This node provides an interface to the localization client.

#### ROKIT Locator Configuration

The bridge looks for ROS parameters under **`/bridge_node/localization_client_config`** and sets these to the ROKIT Locator configuration. This is done only once, during the start of the `bridge_node`.
See the ROKIT Locator API documentation section 8.5.4 for a list of possible options.

To correctly forward the laser scan data, it is important that `ClientSensor.laser.type` is set to `simple`, and that `ClientSensor.laser.address` is set to the IP address (with port) of the computer the bridge is running.

#### Subscribed Topics

* **`/scan`** ([sensor_msgs/LaserScan])

  The laserscan topic of the first laser to translate and forward to the ROKIT Locator.

* **`/scan2`** ([sensor_msgs/LaserScan])

  The laserscan topic of the second laser to translate and forward to the ROKIT Locator.

* **`/odom`** ([nav_msgs/Odometry])

	The odometry topic to translate and forward to the ROKIT Locator. Only used when `enable_odometry` is set to `true`.

* **`/initialpose`** ([geometry_msgs/PoseWithCovarianceStamped])

	Set a seed pose to help localization.
	RViz by default sends a message of this type and topic when the user clicks on the "2D Pose Estimate" button in the toolbar.

#### Published Topics

* **`/bridge_node/client_control_mode`** ([bosch_locator_bridge/ClientControlMode](./msg/ClientControlMode.msg))

	The state of the different modules. See ROKIT Locator API Documentation, chapter 5 "Client Control Mode".

##### Map Creation

* **`/bridge_node/client_map_map`** ([sensor_msgs/PointCloud2])

	The map used for localization as point cloud.

* **`/bridge_node/client_map_visualization`** ([bosch_locator_bridge/ClientMapVisualization](./msg/ClientMapVisualization.msg))

	Describes the current state of the map creation mode.

* **`/bridge_node/client_map_visualization/pose`** ([geometry_msgs/PoseStamped])

	The current pose of the laser sensor during the map creation mode.

* **`/bridge_node/client_map_visualization/scan`** ([sensor_msgs/PointCloud2])

	The current laser scan worked on during the map creation mode.

* **`/bridge_node/client_map_visualization/path_poses`** ([geometry_msgs/PoseArray])

	The path of the laser sensor during the map creation mode.

* **`/bridge_node/client_global_align_visualization`** ([bosch_locator_bridge/ClientGlobalAlignVisualization](./msg/ClientGlobalAlignVisualization.msg))

	Information on global alignment landmarks and their observations.

* **`/bridge_node/client_global_align_visualization/poses`** ([geometry_msgs/PoseArray])

	Previous poses visited by the laser sensor.

* **`/bridge_node/client_global_align_visualization/landmarks/poses`** ([geometry_msgs/PoseArray])

	Poses of the estimated landmarks.

##### Map Recording

* **`/bridge_node/client_recording_map`** ([sensor_msgs/PointCloud2])

	The map as point cloud during the recording mode.

* **`/bridge_node/client_recording_visualization`** ([bosch_locator_bridge/ClientRecordingVisualization](./msg/ClientRecordingVisualization.msg))

	Visualization information for the recording process.

* **`/bridge_node/client_recording_visualization/pose`** ([geometry_msgs/PoseStamped])

	Latest estimated pose of the laser sensor during the recording process.

* **`/bridge_node/client_recording_visualization/scan`** ([sensor_msgs/LaserScan])

	Latest processed laser scan during the recording process.

* **`/bridge_node/client_recording_visualization/path_poses`** ([geometry_msgs/PoseArray])

	Latest path of the laser sensor during the recording process.

##### Localization

* **`/bridge_node/client_localization_map`** ([sensor_msgs/PointCloud2])

	Map as point cloud used during localization.

* **`/bridge_node/client_localization_visualization`** ([bosch_locator_bridge/ClientLocalizationVisualization](./msg/ClientLocalizationVisualization.msg))

	Visualization information for the localization process.

* **`/bridge_node/client_localization_visualization/pose`** ([geometry_msgs/PoseStamped])

	Current estimated pose during localization process.

* **`/bridge_node/client_localization_visualization/scan`** ([sensor_msgs/PointCloud2])

	Currently processed laser scan during localization.

* **`/bridge_node/client_localization_pose`** ([bosch_locator_bridge/ClientLocalizationPose](./msg/ClientLocalizationPose.msg))

	Localization information.

* **`/bridge_node/client_localization_pose/pose`** ([geometry_msgs/PoseStamped])

	6 DoF pose of the laser sensor during localization process.

* **`/bridge_node/client_localization_pose/lidar_odo_pose`** ([geometry_msgs/PoseStamped])

	The current pose of the laser sensor, given in a relative reference frame.

#### Services

* **`/bridge_node/get_config_entry`** ([bosch_locator_bridge/ClientConfigGetEntry](./srv/ClientConfigGetEntry.srv))

	Returns the value of the config entry with the given name.

* **`/bridge_node/start_visual_recording`** ([bosch_locator_bridge/StartRecording](./srv/StartRecording.srv))

	Starts the visual map recording under the given map name.

* **`/bridge_node/stop_visual_recording`** ([std_srvs/Empty])

	Stops visual map recording.

* **`/bridge_node/start_map`** ([bosch_locator_bridge/ClientMapStart](./srv/ClientMapStart.srv))

	Start to create a map from the data contained within a recording.

* **`/bridge_node/stop_map`** ([std_srvs/Empty])

	Stop map creation.

* **`/bridge_node/send_map`** ([bosch_locator_bridge/ClientMapSend](./srv/ClientMapSend.srv))

	Send the given map to the map server. This is a prerequisite so that the map can be used for localization.

* **`/bridge_node/set_map`** ([bosch_locator_bridge/ClientMapSet](./srv/ClientMapSet.srv))

	Set the given map to be used for localization.

* **`/bridge_node/start_localization`** ([std_srvs/Empty])

	Start self-localization within the map.

* **`/bridge_node/stop_localization`** ([std_srvs/Empty])

	Stop self-localization within the map.

### server_bridge_node

This node provides an interface to the map server.

#### Start Server Bridge Node

Start the server bridge node with

    roslaunch bosch_locator_bridge server_bridge.launch bridge_ip:=<HOST_IP> locator_ip:=<LOCATOR_IP> locator_user:=<USER> locator_password:=<PASSWORD>

where
- `<HOST_IP>` is the IP address of the computer the bridge is to be started
- `<LOCATOR_IP>` is the IP address of the computer where the ROKIT Locator is running
- `<USER>` and `<PASSWORD>` are the credentials to log into the ROKIT Locator

#### Services

* **`/server_bridge_node/get_map_with_resolution`** ([bosch_locator_bridge/ServerMapGetImageWithResolution](./srv/ServerMapGetImageWithResolution.srv))

	Stores the requested map in a pair of files (YAML and PNG), as is usual with ROS.

* **`/server_bridge_node/list_server_maps`** ([bosch_locator_bridge/ServerMapList](./srv/ServerMapList.srv))

	Returns list of maps on map server.

## Caveats

### ROKIT Locator closes connection

The ROKIT Locator performs a strict input validation checks and closes the connection, if it receives data that is outside of its specified range.
If you see error messages of this kind in the output of the ROS bridge:
```
Connection reset by peer
```
Double check the data sent and compare it with the acceptable values in the ROKIT Locator API documentation.

Also peek in the LocalizationClient's syslog file (see the ROKIT Locator documentation) for hints (usually `validation of ... failed`).

### Error message: SENSOR_NOT_AVAILABLE

This can happen if you switch the ROKIT Locator into a mode where it requires e.g. laser data, but none is available (e.g. no laser data is sent with a few hundred miliseconds after the mode switch).

To avoid this, make sure `LaserScan` messages are sent to the bridge before switching the ROKIT Locator mode.

## Support of earlier versions of ROKIT Locator

If you have version 1.5 of ROKIT Locator, checkout the corresponding tag:

    git checkout 1.0.8 -b noetic-v1.5

And if you have version 1.4:

    git checkout 1.0.6 -b noetic-v1.4

And if you have version 1.3:

    git checkout 1.0.4 -b noetic-v1.3

And if you have version 1.2:

    git checkout 1.0.2 -b noetic-v1.2


[ROS 1]: https://wiki.ros.org/noetic
[Rexroth ROKIT Locator]: https://www.boschrexroth.com/en/xc/products/product-groups/components-for-mobile-robotics/index
[Poco]: https://pocoproject.org/
[sensor_msgs/LaserScan]: http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
[nav_msgs/Odometry]: http://docs.ros.org/api/sensor_msgs/html/msg/Odometry.html
[std_srvs/Empty]: http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html
[geometry_msgs/PoseWithCovarianceStamped]: http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html
[geometry_msgs/PoseStamped]: http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html
[geometry_msgs/PoseArray]: http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html
[sensor_msgs/PointCloud2]: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
