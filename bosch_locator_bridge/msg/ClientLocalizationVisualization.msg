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

# currently unused; this field is reserved to match a uniqueId provided via the ClientSensorLaser interface
uint64 unique_id

# The localization status
int32 loc_state

# Distance traversed by the sensor since the last successful loop closure
float64 distanceToLastLC

# The relative delay within the system when processing laser scans.
# 0: no delay; 1.0 : delay causes loop closures to be omitted when building the temporary map
float64 delay

# Constants
int32 LOC_STATUS_INFO_LOCALIZATION_NOT_RUNNING = -3
int32 LOC_STATUS_NOT_LOCALIZED_STARTUP = -2
int32 LOC_STATUS_NOT_LOCALIZED = -1
int32 LOC_STATUS_LOCALIZED_ODO_ONLY = 1
int32 LOC_STATUS_LOCALIZED = 2
int32 LOC_STATUS_LOCALIZED_INTERNALUSE1 = 3
int32 LOC_STATUS_LOCALIZED_INTERNALUSE2 = 4
