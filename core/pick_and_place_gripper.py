import sys
sys.path.insert(0, '/home/rokey/Desktop/DTHRC/DTHRC/utils')

import numpy as np
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from pxr import Sdf, UsdPhysics, Gf, UsdGeom  # MOD: use UsdGeom for bolt prim pose
import omni.kit.commands
from omni.physx import get_physx_interface
from rmpflow_controller import RMPFlowController


class RobotController:
    def __init__(self, world, robot,placing_position, bolt_prim_path="/World/Bolt"):  # MOD: allow YOLO-selected bolt prim
        self.world = world
        self.robot =self.world.scene.get_object("my_ur10")
        self.bolt_prim_path = bolt_prim_path  # MOD: track bolt prim path from YOLO
        self.bolt = self.world.scene.get_object("my_bolt") if bolt_prim_path == "/World/Bolt" else None  # MOD
        self.my_controller = None
        self.articulation_controller = None
        print("bolt goood")
        self.stage = world.stage
        self._initial_ee_position = None
        


        self.my_controller = RMPFlowController(
            name="my_ur10_cspace_controller", 
            robot_articulation=self.robot,
            attach_gripper = True
        )
        self.articulation_controller = self.robot.get_articulation_controller()

        self.task_phase = 1
        self.joint_created = False
        try: 
            self._placing_position = placing_position # 볼트 박스 위치 np.array([-1.25, -0.25047, 1.5])
        except Exception as e:
            print(f"no place position: {e}")
            
        try:
            ee_pose, _ = self.robot.gripper.get_world_pose()
            self._initial_ee_position = np.array(ee_pose, dtype=np.float64)
        except Exception as e:
            print(f"no initial ee position: {e}")

    def set_bolt_prim_path(self, bolt_prim_path):
        if not bolt_prim_path:
            print('no bolt prim path')
            return
        if bolt_prim_path == self.bolt_prim_path:
            return
        self.bolt_prim_path = bolt_prim_path  # MOD: update target bolt from YOLO
        self.bolt = self.world.scene.get_object("my_bolt") if bolt_prim_path == "/World/Bolt" else None  # MOD
        print(f"target bolt prim path: {self.bolt_prim_path}")

    def control_robot(self, target_bolt_prim_path=None, speed_ratio=1.0):
        if speed_ratio == 0.0:
            print("[safety] stop mode: human closed")
            # 로봇을 즉시 멈추기 위해 현재 관절 속도를 0으로 설정
            self.robot.set_joint_velocities(np.zeros_like(self.robot.get_joint_velocities()))
            # self._sync_bolt_to_gripper()
            return

        if speed_ratio == 0.15:
            print("[safety] slow mode: human closed")
        
        # 1. 현재 정보 업데이트
        if target_bolt_prim_path:
            self.set_bolt_prim_path(target_bolt_prim_path)  # MOD
        ee_pose, _ = self.robot.gripper.get_world_pose()
        bolt_pose = self._get_bolt_world_position()  # MOD: use YOLO-selected bolt prim
        if bolt_pose is None:
            print("bolt prim not ready")  # MOD
            return
        print(bolt_pose)
        
        # 2. 페이즈별 로직 (State Machine)
        if self.task_phase == 1: # 볼트 접근 감시
            bolt_pose[2] += 0.035    # 볼트보다 0.035 높은 위치를 잡음
            if bolt_pose[1] <= 1.1:  #로봇이 pick하기 시작하는 시점, 튜닝 필요
                print(f"close bolt: {self.bolt_prim_path}")
                self.task_phase = 2


        elif self.task_phase == 2: # 볼트로 이동
            print(f"task_phase :{self.task_phase} {self.bolt_prim_path} access")
            target_pos = self._get_bolt_world_position()  # MOD: target from selected bolt
            if target_pos is None:
                return

            target_ori = euler_angles_to_quat(np.array([0, np.pi/2, 0]))    # 그리퍼가 접근하는 각도
            action = self._apply_rmp_move(target_pos, target_ori, speed_ratio=speed_ratio)    # 로봇이 타겟으로 이동
            dist = np.linalg.norm(ee_pose - bolt_pose)
            print(f"distance to bolt: {dist}")

            # 엔드 이펙터 위치와 볼트 위치가 가까워지면 fixed joint 생성, 다음 페이즈로 이동
            if dist < 0.17:  # 0.17: 볼트가 정지해있는 상황 튜닝 필요
                print("############### close gripper ###############")
                self.robot.gripper.close()
                bolt_pose = self._get_bolt_world_position()
                self.bolt_pose_up = np.array([bolt_pose[0], bolt_pose[1], bolt_pose[2]+0.5])  # 기존 볼트 위치보다 0.3m 위까지 올리기 위한 위치 저장

                self.task_phase = 3

        elif self.task_phase == 3: # 들어올리기
            print(f"task_phase :{self.task_phase} {self.bolt_prim_path} bolt up")
            # 매순간 볼트, 엔드 이펙터 위치 가져오기
            bolt_pose = self._get_bolt_world_position()  # MOD
            if bolt_pose is None:
                return
            ee_pose = self.robot.gripper.get_world_pose()[0]

            action = self._apply_rmp_move(self.bolt_pose_up, euler_angles_to_quat(np.array([0, np.pi/2, 0])), speed_ratio=speed_ratio)
            
            if ee_pose[2] > self.bolt_pose_up[2]:   # 로봇팔 위치가 1.5를 넘으면 다음 페이즈로 이동
                self.my_controller.reset()      ###### 추가 ########
                self.task_phase = 3.5
            
            ## 원래 코드
            current_joint_positions = self.robot.get_joint_positions()
            
            if np.all(np.abs(current_joint_positions[:6] - action.joint_positions) < 0.001):
                self.my_controller.reset()
                self.task_phase = 3.5

        elif self.task_phase == 3.5: # 목표 지점 이동 전 중간 위치
            print(f"task_phase :{self.task_phase} {self.bolt_prim_path} mid move")
            if self._move_to_initial_position(speed_ratio):
                self.my_controller.reset()
                self.task_phase = 4
        
        elif self.task_phase == 4: # 목표 지점으로 이동
            print(f"task_phase :{self.task_phase} {self.bolt_prim_path} placing")
            bolt_pose = self._get_bolt_world_position()  # MOD
            if bolt_pose is None:
                return
            
            action = self._apply_rmp_move(self._placing_position, euler_angles_to_quat(np.array([0, np.pi/2, 0])), speed_ratio=speed_ratio)

            current_joint_positions = self.robot.get_joint_positions()

            # 원래 페이즈 변경 코드
            if np.all(np.abs(current_joint_positions[:6] - action.joint_positions) < 0.001):
                self.my_controller.reset()
                self.task_phase = 5

            if np.linalg.norm(bolt_pose - self._placing_position) < 0.05:
                self.task_phase = 5

        elif self.task_phase == 5: # 조인트 해제 및 종료
            print(f"task_phase :{self.task_phase} {self.bolt_prim_path} finish up")
            self.robot.gripper.open()
            self.task_phase = 5.5

        elif self.task_phase == 5.5: # 중간 위치로 복귀 후 다음 pick
            print(f"task_phase :{self.task_phase} {self.bolt_prim_path} return home")
            if self._move_to_initial_position(speed_ratio):
                self.my_controller.reset()
                self.task_phase = 6

    # 로봇이 target으로 이동하는 함수
    def _apply_rmp_move(self, pos, ori, speed_ratio=1.0):
        action = self.my_controller.forward(
            target_end_effector_position=pos,
            target_end_effector_orientation=ori
        )

        if action.joint_velocities is not None:
            action.joint_velocities *= speed_ratio
            
        self.robot.apply_action(action)
        return action
    
    def _move_to_initial_position(self, speed_ratio=1.0):
        ee_pose = self.robot.gripper.get_world_pose()[0]
        mid_position = self._initial_ee_position
        if mid_position is None:
            mid_position = np.array(ee_pose, dtype=np.float64)
            self._initial_ee_position = mid_position

        action = self._apply_rmp_move(
            mid_position,
            euler_angles_to_quat(np.array([0, np.pi/2, 0])),
            speed_ratio=speed_ratio,
        )
        current_joint_positions = self.robot.get_joint_positions()
        if np.all(np.abs(current_joint_positions[:6] - action.joint_positions) < 0.001):
            return True
        if np.linalg.norm(ee_pose - mid_position) < 0.05:
            return True
        return False


    def _get_bolt_prim(self):
        if not self.bolt_prim_path:
            return None
        bolt_prim = self.stage.GetPrimAtPath(self.bolt_prim_path)  # MOD
        if not bolt_prim.IsValid():
            return None
        return bolt_prim

    def _get_bolt_world_position(self):
        if self.bolt is not None:
            bolt_pose, _ = self.bolt.get_world_poses()
            return bolt_pose[0] if len(bolt_pose) > 0 else None
        bolt_prim = self._get_bolt_prim()
        if bolt_prim is None:
            return None
        xform_cache = UsdGeom.XformCache()  # MOD
        transform = xform_cache.GetLocalToWorldTransform(bolt_prim)
        translation = transform.ExtractTranslation()
        # return np.array([translation[0], translation[1], translation[2]-0.05], dtype=np.float64)  # MOD
        return np.array([translation[0], translation[1], translation[2]], dtype=np.float64)
