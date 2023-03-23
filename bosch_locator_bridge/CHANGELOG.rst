^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bosch_locator_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.9 (2023-03-23)
------------------
* Try to stop everything before setting config list (`#35 <https://github.com/boschglobal/locator_ros_bridge/issues/35>`_)
* Add errorFlags and infoFlags fields to ClientLocalizationPose (`#33 <https://github.com/boschglobal/locator_ros_bridge/issues/33>`_)
* Update to ROKIT Locator version 1.6
* Contributors: Stefan Laible

2.1.8 (2023-02-14)
------------------
* Support setting arbitrary Locator config entries
* Update module versions in server node
* Use mutex to just make one json_rpc_call at a time
* Contributors: Fabian König, Stefan Laible

2.1.7 (2022-09-08)
------------------
* Fixed a bug that could cause latency in localization poses
* Remove tf broadcaster
* Check if laser scan message is valid
* Add refresh timer to service callback group to avoid overlapping json rpc calls
* Update to ROKIT Locator version 1.5
* Contributors: Fabian König, Stefan Laible

2.1.6 (2022-06-13)
------------------
* fix module version check for compatibility with v1.4.0
* add parameter to enable intensities
* Contributors: Fabian König, Stefan Laible

2.1.5 (2022-02-16)
------------------
* update to ROKIT Locator version 1.4
* add interface for second laser
* Contributors: Stefan Laible

2.1.4 (2022-01-24)
------------------
* handle NaNs in range values
* add service for getting config entries
* add server interface
* Contributors: Stefan Laible, robertwil

2.1.3 (2021-09-02)
------------------
* update to ROKIT Locator version 1.3
* Contributors: Stefan Laible

2.1.2 (2021-07-22)
------------------
* guarantee that duration_rotate is greater zero
* Contributors: Stefan Laible

2.1.1 (2021-05-19)
------------------
* add tf2_geometry_msgs dependency
* Contributors: Stefan Laible

2.1.0 (2021-05-17)
------------------
* initial version
* Contributors: Stefan Laible
