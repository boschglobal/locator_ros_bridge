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

# A set of landmarks known to the system
bosch_locator_bridge/ClientGlobalAlignLandmarkVisualizationInformation[] landmarks

#Specifies which landmarks in the landmarks array were observed from which poses in
#the poses array. Note that some landmarks may have never been observed, and
#that some poses within the poses array may not be associated with any landmark.
bosch_locator_bridge/ClientGlobalAlignLandmarkObservationNotice[] observations
