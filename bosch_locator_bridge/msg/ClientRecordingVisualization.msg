# Copyright (c) 2021 - for information on the respective copyright owner
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

# The time at which the Localization Client received the laser scan used to generate this Datagram
time timestamp

# A unique id for matching datagrams that were emitted at the same time but by different visualization interfaces
uint64 visualization_id

# The ClientRecordingRecordingStatus
int32 status

# Distance traversed by the sensor since the last successful loop closure
float64 distanceToLastLC

# The relative delay within the system when processing laser scans.
# 0: no delay; 1.0 : delay causes loop closures to be omitted when building the temporary map
float64 delay

# Currently unused
float64 progress

# Please note that pose, scan, and path_poses is published directly via the standard message types (custom nested messages would hinder using ros tools, such as rviz)

#  Array of order state_1, state_2, using the ClientRecordingPathTypeEnum
int32[] path_types


# Constants
int32 CLIENT_RECORDING_RECORDING_STATUS_STARTUP = -2
int32 CLIENT_RECORDING_RECORDING_STATUS_DELAYED = 1
int32 CLIENT_RECORDING_RECORDING_STATUS_OK = 2

int32 CLIENT_RECORDING_PATH_TYPE_ENUM_NORMAL = 0
int32 CLIENT_RECORDING_PATH_TYPE_ENUM_POSSIBLE_LOOP_CLOSURE = 1
int32 CLIENT_RECORDING_PATH_TYPE_ENUM_POSSIBLY_VISIBLE_LOOP_CLOSURE = 2
