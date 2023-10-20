^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bosch_locator_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.10 (2023-10-20)
-----------
* Update to ROKIT Locator version 1.6
* Document the map expansion workflow
* Add rviz config files for map expansion
* Add services and interfaces for map expansion
* Add expandmap_state in control mode
* Remove field distanceToLastLC in ClientLocalizationVisualizationDatagram
* Introduce parameter odometry_velocity_set of odometry message
* Configurable Locator ports
* Contributors: Sheung Ying Yuen-Wille

1.0.9 (2023-03-21)
------------------
* Try to stop everything before setting config list (`#38 <https://github.com/boschglobal/locator_ros_bridge/issues/38>`_)
* Add errorFlags and infoFlags fields to ClientLocalizationPose (`#32 <https://github.com/boschglobal/locator_ros_bridge/issues/32>`_)
* Update to ROKIT Locator version 1.6
* Contributors: Sheung Ying Yuen-Wille, Stefan Laible

1.0.8 (2023-01-30)
------------------
* Support arrays in config for server node
* Update module versions in server node
* Use mutex to just make one json_rpc_call at a time
* Contributors: Stefan Laible

1.0.7 (2022-09-08)
------------------
* Fixed a bug that could cause latency in localization poses
* Remove tf broadcaster
* Support arrays in config
* Check if laser scan message is valid
* Update to ROKIT Locator version 1.5
* Contributors: Fabian König, Stefan Laible

1.0.6 (2022-04-13)
------------------
* Fix check of locator config params
* Change version numbers for compatibility with v1.4.0
* Add parameters for using intensities
* Contributors: Stefan Laible

1.0.5 (2022-02-15)
------------------
* update to ROKIT Locator version 1.4
* add interface for second laser
* Contributors: Stefan Laible

1.0.4 (2022-01-24)
------------------
* handle NaNs in range values
* add service for getting config entries
* add server interface
* Contributors: Stefan Laible, robertwil

1.0.3 (2021-09-02)
------------------
* update to ROKIT Locator version 1.3
* Contributors: Stefan Laible

1.0.2 (2021-07-22)
------------------
* guarantee that duration_rotate is greater zero
* Contributors: Stefan Laible

1.0.1 (2021-05-20)
------------------
* bump minimum cmake version
* add poco dependency
* Contributors: Stefan Laible

1.0.0 (2021-05-17)
------------------
* initial version
* Contributors: Stefan Laible
