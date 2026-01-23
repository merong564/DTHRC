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
import omni

import isaacsim.core.api.tasks as tasks
import numpy as np
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.prims import SingleXFormPrim
from omni.isaac.core.world import World
from pxr import Usd, UsdPhysics

# Inheriting from the base class Follow Target
# class FollowTargetCustom(tasks.FollowTarget):
#     def __init__(self, name, target_position, robot_prim_path):
#         ## 수정 ##
#         self._robot_prim_path = robot_prim_path  # USD 내 로봇 경로 (예: /World/ur10e)
#         # tasks.FollowTarget.__init__(
#         #     self,
#         #     name=name,
#         #     target_position=target_position
#         # )

#         # [해결 핵심 1] 부모 클래스 초기화 전에 로봇 객체를 미리 생성하여 할당합니다.
#         # 이렇게 하면 get_params()가 내부적으로 호출되어도 self._robot.name에 접근할 수 있습니다.
#         self._robot = self.set_robot() 

#         # [해결 핵심 2] 부모 클래스 초기화 시 target_prim_path를 명시적으로 넘겨줍니다.
#         tasks.FollowTarget.__init__(
#             self,
#             name=name,
#             target_prim_path="/World/bolt", # 실제 타겟 경로 전달
#             target_name="target_bolt",
#             target_position=target_position
#         )

#     def set_robot(self) -> SingleManipulator:
#         # [수정] 새 USD를 로드하지 않고, 기존 prim_path를 사용하여 객체 생성
#         manipulator = SingleManipulator(
#             prim_path=self._robot_prim_path,
#             name="ur10e_robot",
#             # end_effector 경로는 USD 구조에 맞춰 확인 필요 (보통 ee_link 등)
#             end_effector_prim_path=f"{self._robot_prim_path}/ee_link" 
#         )

#         # stage = omni.usd.get_context().get_stage()
#         # # 에러가 발생하는 그리퍼 베이스 경로
#         # gripper_base_path = f"{self._robot_prim_path}/ee_link/SurfaceGripper"
#         # gripper_prim = stage.GetPrimAtPath(gripper_base_path)

#         # if gripper_prim.IsValid():
#         #     # 그리퍼 베이스를 포함하여 그 하위의 모든 자식 노드를 탐색
#         #     for prim in Usd.PrimRange(gripper_prim):
#         #         # RigidBodyAPI가 적용되어 있는지 확인
#         #         if prim.HasAPI(UsdPhysics.RigidBodyAPI):
#         #             omni.kit.commands.execute(
#         #                 "RemovePhysicsComponentCommand",
#         #                 usd_prim=prim,
#         #                 component="PhysicsRigidBodyAPI"
#         #             )
#         #     print(f"Cleanup complete: All overlapping RigidBodies removed from {gripper_base_path}")
#         return manipulator
    
#     def set_up_scene(self, scene):
#         super().set_up_scene(scene)

    
# #     def set_params(
# #     self,
# #     target_prim_path: Optional[str] = "/World/bolt", # 예시 경로
# #     target_name: Optional[str] = "target_bolt",
# #     target_position: Optional[np.ndarray] = None,
# #     target_orientation: Optional[np.ndarray] = None,
# # ) -> None:
# #         scene = World().scene
    
# #         if target_prim_path is not None:
# #             # 1. 기존 타겟 제거 로직 (기존과 동일)
# #             if self._target is not None:
# #                 if self._target.name in self._task_objects:
# #                     del self._task_objects[self._target.name]

# #             # 2. 핵심: 이미 존재하는 프림(/World/bolt)을 SingleXFormPrim으로 래핑
# #             # 이 클래스는 새로운 프림을 만들기도 하지만, 경로가 존재하면 해당 프림을 제어 대상으로 잡습니다.
# #             self._target = scene.add(
# #                 SingleXFormPrim(
# #                     prim_path=target_prim_path,
# #                     name=target_name,
# #                     position=target_position,
# #                     orientation=target_orientation,
# #                 )
# #             )

# #             # 3. 태스크 오브젝트 등록
# #             self._task_objects[self._target.name] = self._target
            
                    
# #         else:
# #             # 타겟이 이미 설정된 상태에서 위치만 바꿀 때
# #             self._target.set_local_pose(position=target_position, orientation=target_orientation)
    
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
    def __init__(
        self,
        name: str = "ur10e_follow_target",
        target_prim_path: Optional[str] = "/World/bolt",
        target_name: Optional[str] = "bolt_target",
        target_position: Optional[np.ndarray] = None,
        target_orientation: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
    ) -> None:
        tasks.FollowTarget.__init__(
            self,
            name=name,
            target_prim_path=target_prim_path,
            target_name=target_name,
            target_position=target_position,
            target_orientation=target_orientation,
            offset=offset,
        )

        return
    
    def set_robot(self):
        # EnvManager에서 이미 scene.add()를 했으므로, Scene에서 이름으로 찾아옵니다.
        if self.scene.object_exists(self._robot_name):
            return self.scene.get_object(self._robot_name)
        else:
            # 혹시라도 Scene에 없다면 에러를 발생시키거나 새로 생성하는 로직이 필요합니다.
            raise RuntimeError(f"Robot with name {self._robot_name} not found in scene.")

    # def set_robot(self) -> SingleManipulator:

    #     assets_root_path = get_assets_root_path()
    #     if assets_root_path is None:
    #         raise Exception("Could not find Isaac Sim assets folder")
    #     asset_path = (
    #         assets_root_path + "/Isaac/Samples/Rigging/Manipulator/configure_manipulator/ur10e/ur/ur_gripper.usd"
    #     )
    #     add_reference_to_stage(usd_path=asset_path, prim_path="/ur")
    #     # define the gripper
    #     gripper = ParallelGripper(
    #         # We chose the following values while inspecting the articulation
    #         end_effector_prim_path="/ur/ee_link/robotiq_arg2f_base_link",
    #         joint_prim_names=["finger_joint"],
    #         joint_opened_positions=np.array([0]),
    #         joint_closed_positions=np.array([40]),
    #         action_deltas=np.array([-40]),
    #         use_mimic_joints=True,
    #     )
    #     # define the manipulator
    #     manipulator = SingleManipulator(
    #         prim_path="/ur",
    #         name="ur10_robot",
    #         end_effector_prim_path="/ur/ee_link/robotiq_arg2f_base_link",
    #         gripper=gripper,
    #     )
    #     return manipulator
    
    def get_params(self) -> dict:
        """
        부모 클래스의 get_params를 호출하기 전에 
        현재 self._target의 상태를 출력합니다.
        """
        print("-" * 50)
        print("[FollowTargetCustom.get_params] 내부 확인")
        
        # 1. self._target 객체 자체 확인
        print(f"-> 현재 self._target 객체: {self._target}")
        
        if self._target is not None:
            # 2. 객체가 존재한다면 세부 정보 출력
            print(f"-> target 프림 경로(prim_path): {self._target.prim_path}")
            print(f"-> target 이름(name): {self._target.name}")
        else:
            # 3. None일 경우 경고 출력
            print("-> [경고] self._target이 아직 None입니다. set_up_scene이 정상 실행되었는지 확인하세요.")
        
        print("-" * 50)
        
        # 부모 클래스(tasks.FollowTarget)의 get_params를 실행하여 결과값 리턴
        return super().get_params()
    

    def set_up_scene(self, scene) -> None:
        self._scene = scene
        
        # 2. 로봇 설정 및 씬 등록
        self._robot = self.set_robot()
        scene.add(self._robot) # 이미 존재하더라도 Scene API에 등록해야 get_observations 등이 작동함

        # 3. 타겟 설정 (이미 존재하는 타겟 prim_path 사용)
        # set_params 내부에서 is_prim_path_valid를 체크하므로 기존 경로를 그대로 넘김
        self.set_params(
            target_prim_path=self._target_prim_path,
            target_name=self._target_name or "target",
            target_position=self._target_position,
            target_orientation=self._target_orientation
        )
        # 부모의 set_up_scene을 먼저 실행 (이 안에서 set_params가 실행됨)
        # super().set_up_scene(scene)

        # 2. ★ 여기가 확인 포인트입니다! ★
        # 부모 클래스가 /World/bolt를 찾아서 self._target에 할당한 직후입니다.
        print("-" * 30)
        print(f"[검증] self._target 값: {self._target}")
        if self._target is not None:
            print(f"[검증] target prim_path: {self._target.prim_path}")
            print(f"[검증] target 객체 타입: {type(self._target)}")
        else:
            print("[검증] 에러: /World/bolt를 찾지 못해 target이 여전히 None입니다.")
        print("-" * 30)
        return