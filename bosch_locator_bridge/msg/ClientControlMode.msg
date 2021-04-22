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

# Timestamp of the update
time stamp

# This mode is in RUN-state while a map is calculated live during the recording
uint8 visual_recording_state

# While a map is generated from a recording the ClientState of the Map mode is RUN
uint8 map_state

# This mode is in RUN-state while the system localizes itself to a map received from the Map Server
uint8 localization_state

#  This mode is in RUN-state while a recording is made
uint8 recording_state

# This mode is RUN-state while the user works on aligning the map
uint8 alignment_state

# This mode is RUN-state while the user is creating a mask for the laser scans
uint8 mask_state


# Constants
uint8 CLIENT_CONTROL_STATE_INIT=0
uint8 CLIENT_CONTROL_STATE_READY=1
uint8 CLIENT_CONTROL_STATE_RUN=2
uint8 CLIENT_CONTROL_STATE_NOT_AVAILABLE=4
