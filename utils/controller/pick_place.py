# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import isaacsim.robot.manipulators.controllers as manipulators_controllers
from isaacsim.core.prims import SingleArticulation
from isaacsim.robot.manipulators.grippers import SurfaceGripper

from .rmpflow import RMPFlowController


class PickPlaceController(manipulators_controllers.PickPlaceController):
    def __init__(
        self, name: str, gripper: SurfaceGripper, robot_articulation: SingleArticulation, events_dt=None
    ) -> None:
        # [참고] Surface Gripper는 흡착을 위해 대기 시간이 필요할 수 있으므로 
        # events_dt는 기본값을 사용하거나 필요시 조정합니다.
        if events_dt is None:
            events_dt = [0.005, 0.002, 1, 0.05, 0.0008, 0.005, 0.0008, 0.1, 0.0008, 0.008]
            
        manipulators_controllers.PickPlaceController.__init__(
            self,
            name=name,
            cspace_controller=RMPFlowController(
                name=name + "_cspace_controller", robot_articulation=robot_articulation
            ),
            gripper=gripper,
            events_dt=events_dt,
            end_effector_initial_height=0.6,
        )