# Copyright (c) 2023 - for information on the respective copyright owner
# see the NOTICE file and/or the repository https://github.com/boschglobal/locator_ros_bridge.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# The time at which the Localization Client received the ClientExpandMapVisualizationDatagram
time timestamp

# A unique id for matching datagrams that were emitted at the same time but by different visualization interfaces
uint64 visualization_id

# A set of OverwriteZones
bosch_locator_bridge/ClientExpandMapOverwriteZoneInformation[] zones

# The poses in the prior map
geometry_msgs/PoseArray prior_map_poses

# The type of the poses in the prior map
int32[] prior_map_pose_types
