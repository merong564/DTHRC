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


from typing import Optional

import isaacsim.core.api.tasks as tasks
import numpy as np
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.storage.native import get_assets_root_path


# Inheriting from the base class Follow Target
class FollowTargetCustom(tasks.FollowTarget):
    def __init__(self, name, target_position, robot_prim_path):
        self._robot_prim_path = robot_prim_path  # USD 내 로봇 경로 (예: /World/ur10e)
        tasks.FollowTarget.__init__(
            self,
            name=name,
            target_position=target_position
        )

    def set_robot(self) -> SingleManipulator:
        # [수정] 새 USD를 로드하지 않고, 기존 prim_path를 사용하여 객체 생성
        manipulator = SingleManipulator(
            prim_path=self._robot_prim_path,
            name="ur10e_robot",
            # end_effector 경로는 USD 구조에 맞춰 확인 필요 (보통 ee_link 등)
            end_effector_prim_path=f"{self._robot_prim_path}/ee_link" 
        )
        return manipulator
