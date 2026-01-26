import sys
sys.path.insert(0, '/home/rokey/Desktop/DTHRC/DTHRC/utils')

import numpy as np
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from pxr import Sdf, UsdPhysics, Gf, UsdGeom  # MOD: use UsdGeom for bolt prim pose
import omni.kit.commands
from omni.physx import get_physx_interface
from rmpflow_controller import RMPFlowController

<<<<<<< HEAD

class RobotController:
    def __init__(self, world, robot,placing_position, bolt_prim_path="/World/Bolt"):  # MOD: allow YOLO-selected bolt prim
=======
n = 0
nn=0
class RMPFlowController(mg.MotionPolicyController):

    def __init__(
        self,
        name: str,
        robot_articulation: SingleArticulation,
        physics_dt: float = 1.0 / 60.0,
        attach_gripper: bool = False,
    ) -> None:

        if attach_gripper:
            self.rmp_flow_config = mg.interface_config_loader.load_supported_motion_policy_config(
                "UR10", "RMPflowSuction"
            )
        else:
            self.rmp_flow_config = mg.interface_config_loader.load_supported_motion_policy_config("UR10", "RMPflow")
        self.rmp_flow = mg.lula.motion_policies.RmpFlow(**self.rmp_flow_config)

        self.articulation_rmp = mg.ArticulationMotionPolicy(robot_articulation, self.rmp_flow, physics_dt)

        mg.MotionPolicyController.__init__(self, name=name, articulation_motion_policy=self.articulation_rmp)
        (
            self._default_position,
            self._default_orientation,
        ) = self._articulation_motion_policy._robot_articulation.get_world_pose()
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position, robot_orientation=self._default_orientation
        )
        return

    def reset(self):
        mg.MotionPolicyController.reset(self)
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position, robot_orientation=self._default_orientation
        )

class RobotController:
    def __init__(self, world, robot,placing_position,target_obj):
>>>>>>> feature/safety-system
        self.world = world
        self.robot = robot
        self.robot =self.world.scene.get_object("my_ur10")
<<<<<<< HEAD
        self.bolt_prim_path = bolt_prim_path  # MOD: track bolt prim path from YOLO
        self.bolt = self.world.scene.get_object("my_bolt") if bolt_prim_path == "/World/Bolt" else None  # MOD
=======
>>>>>>> feature/safety-system
        self.my_controller = None
        self.articulation_controller = None
        self.stage = world.stage
        self.set_target(target_obj)


        self.my_controller = RMPFlowController(
            name="my_ur10_cspace_controller", 
            robot_articulation=self.robot,
            attach_gripper = True
        )
        self.articulation_controller = self.robot.get_articulation_controller()

        self.task_phase = 1
        self.joint_created = False
        try: 
            #self._placing_position = placing_position
            if "bolt" in target_obj.name.lower(): 
                self._placing_position = np.array([-0.90365, -0.25047, 1.5])
            elif "nut" in target_obj.name.lower():
                self._placing_position = np.array([-0.90365, -0.88664, 1.5]) # 볼트 박스 위치 np.array([-1.25, -0.25047, 1.5])
        except Exception as e:
            print(f"no place position: {e}")

<<<<<<< HEAD
    def set_bolt_prim_path(self, bolt_prim_path):
        if not bolt_prim_path:
            print('no bolt prim path')
            return
        if bolt_prim_path == self.bolt_prim_path:
            return
        self.bolt_prim_path = bolt_prim_path  # MOD: update target bolt from YOLO
        self.bolt = self.world.scene.get_object("my_bolt") if bolt_prim_path == "/World/Bolt" else None  # MOD
        print(f"target bolt prim path: {self.bolt_prim_path}")

    def control_robot(self, target_bolt_prim_path=None):
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
            if bolt_pose[0] >= 0.5:           # 0.5: 로봇이 pick하기 시작하는 시점, 튜닝 필요
                print("close bolt")
                self.task_phase = 3
=======
    def set_target(self, target_obj):
        self.target = target_obj

        if "bolt" in target_obj.name.lower(): 
            self.target_path = "/World/Bolt"
            self._placing_position = np.array([-0.90365, -0.25047, 1.5])
        else:
            self.target_path = "/World/Nut"
            self._placing_position = np.array([-0.90365, -0.88664, 1.5])
        
        self.task_phase = 1 # 페이즈 리셋
        self.joint_created = False # 조인트 상태 리셋 필수

    def control_robot(self,speed_ratio=1.0):
        global n 
        global nn
        if self.task_phase ==9:
            return
        
        ee_pose, _ = self.robot.gripper.get_world_pose()
        target_pose, _ = self.target.get_world_poses()
        target_pose = target_pose[0]
        # 1. 현재 정보 업데이트
        if n<=60:
            n+=1
        else:
            if speed_ratio == 0.0:
            # 로봇을 즉시 멈추기 위해 현재 관절 속도를 0으로 설정
                self.robot.set_joint_velocities(np.zeros_like(self.robot.get_joint_velocities()))
                # self._sync_bolt_to_gripper()
                return
        
        # 2. 페이즈별 로직 (State Machine)
        if self.task_phase == 1: # 볼트 접근 감시
            if "bolt" in self.target.name.lower(): 
                target_pose[2] += 0.035
            else:
                target_pose[2] += 0.02
>>>>>>> feature/safety-system

            if target_pose[1] <= 1.1:           # 0.5: 로봇이 pick하기 시작하는 시점, 튜닝 필요
                print("close bolt")
                self.task_phase = 2

        # pick 대기 시간 주기
        elif self.task_phase == 2:
            if "bolt" in self.target.name.lower(): 
                if nn <150:
                    action = self._apply_rmp_move(np.array([0.7, -0.34, 1.2]), euler_angles_to_quat(np.array([0, np.pi/2, 0])),speed_ratio=speed_ratio)
                    nn+=1
                else:
                    self.task_phase = 3
            else:
                if nn <150:
                    action = self._apply_rmp_move(np.array([0.7, -0.34, 1.2]), euler_angles_to_quat(np.array([0, np.pi/2, 0])),speed_ratio=speed_ratio)
                    nn+=1
                else:
                    self.task_phase = 3

        elif self.task_phase == 3: # 볼트로 이동
<<<<<<< HEAD
            print(f"task_phase :{self.task_phase} bolt access")
            target_pos = self._get_bolt_world_position()  # MOD: target from selected bolt
            if target_pos is None:
                return
            
            # print(f'taget pos: {target_pos}')
=======
            target_pos = self.target.get_world_poses()[0][0]       # 매순간 볼트의 위치를 가져와 타겟으로 설정
>>>>>>> feature/safety-system

            target_ori = euler_angles_to_quat(np.array([0, np.pi/2, 0]))    # 그리퍼가 접근하는 각도
            
            action = self._apply_rmp_move(target_pos, target_ori,speed_ratio=speed_ratio)    # 로봇이 타겟으로 이동
            
<<<<<<< HEAD
            dist = np.linalg.norm(ee_pose - bolt_pose)
            print(f'dist: {dist}')

            # 엔드 이펙터 위치와 볼트 위치가 가까워지면 fixed joint 생성, 다음 페이즈로 이동
            if dist < 0.18:  # 0.18 튜닝 필요
                # print(f"Distance: {dist:.4f}m - Creating Fixed Joint!")
                # self._create_fixed_joint()
            ######### 추가 #########
            # current_joint_positions = self.robot.get_joint_positions()
            # if np.all(np.abs(current_joint_positions[:6] - action.joint_positions) < 0.001):
                print("############### close gripper ###############")
                self.robot.gripper.close()
                bolt_pose = self._get_bolt_world_position()
                self.bolt_pose_up = np.array([bolt_pose[0], bolt_pose[1], bolt_pose[2]+0.3])  # 기존 볼트 위치보다 0.3m 위까지 올리기 위한 위치 저장

=======
            dist = np.linalg.norm(ee_pose - target_pose)
            
            if dist < 0.25:  # 0.25 튜닝 필요
>>>>>>> feature/safety-system
                self.task_phase = 6


        elif self.task_phase == 6: # 들어올리기
            # 매순간 볼트, 엔드 이펙터 위치 가져오기
<<<<<<< HEAD
            bolt_pose = self._get_bolt_world_position()  # MOD
            if bolt_pose is None:
                return
            ee_pose = self.robot.gripper.get_world_pose()[0]

            # print(f"task_phase :{self.task_phase} picking: bolt z up")
            # target_pos = np.array([bolt_pose[0], bolt_pose[1], bolt_pose[2]+0.05])
            ## 추후 수정해보기: 로봇팔 움직인 후에 볼트 위치 변경
            action = self._apply_rmp_move(self.bolt_pose_up, euler_angles_to_quat(np.array([0, np.pi/2, 0])))
=======
            target_pose = self.target.get_world_poses()[0][0]
            ee_pose = self.robot.gripper.get_world_pose()[0]
            target_pos = np.array([target_pose[0], target_pose[1], target_pose[2]+0.05])
            ## 추후 수정해보기: 로봇팔 움직인 후에 볼트 위치 변경
            action = self._apply_rmp_move(target_pos, euler_angles_to_quat(np.array([0, np.pi/2, 0])),speed_ratio=speed_ratio)
>>>>>>> feature/safety-system
            
            # self._sync_bolt_to_gripper()  # 볼트 위치 이동
            
<<<<<<< HEAD
            if ee_pose[2] > self.bolt_pose_up[2]:   # 로봇팔 위치가 1.5를 넘으면 다음 페이즈로 이동
            # current_joint_positions = self.robot.get_joint_positions()
            # if np.all(np.abs(current_joint_positions[:6] - action.joint_positions) < 0.001):
                self.my_controller.reset()      ###### 추가 ########
=======
            if ee_pose[2] > 1.4:   # 로봇팔 위치가 1.5를 넘으면 다음 페이즈로 이동
>>>>>>> feature/safety-system
                self.task_phase = 7
            
            # 원래 코드
            current_joint_positions = self.robot.get_joint_positions()
            
            if np.all(np.abs(current_joint_positions[:6] - action.joint_positions) < 0.001):
                self.my_controller.reset()
                self.task_phase = 7
        
<<<<<<< HEAD
        elif self.task_phase == 7: # 목표 지점으로 이동
            print(f"task_phase :{self.task_phase} placing")
            bolt_pose = self._get_bolt_world_position()  # MOD
            if bolt_pose is None:
                return
            #self._placing_position = 
            action = self._apply_rmp_move(self._placing_position, euler_angles_to_quat(np.array([0, np.pi/2, 0])))
            # self._sync_bolt_to_gripper()
=======
        elif self.task_phase == 7:
            self._set_bolt_visibility(False)
            target_pose = self.target.get_world_poses()[0][0]
            #self._placing_position = 
            action = self._apply_rmp_move(self._placing_position, euler_angles_to_quat(np.array([0, np.pi/2, 0])),speed_ratio=speed_ratio)
            self._sync_bolt_to_gripper()
>>>>>>> feature/safety-system

            current_joint_positions = self.robot.get_joint_positions()
            # 원래 페이즈 변경 코드
            if np.all(np.abs(current_joint_positions[:6] - action.joint_positions) < 0.001):
                self.my_controller.reset()
                self.task_phase = 8

            if np.linalg.norm(target_pose - self._placing_position) < 0.1:
                self.task_phase = 8

<<<<<<< HEAD
        elif self.task_phase == 8: # 조인트 해제 및 종료
            print(f"task_phase :{self.task_phase} finish up")
            self.robot.gripper.open()

            # if self.joint_created:
            #     self._remove_fixed_joint()
            
            # self.stage = omni.usd.get_context().get_stage()
            # x1,y1,z1=self.robot.gripper.get_world_pose()[0]  # 함수 실행될 때마다 그리퍼 위치 가져오기
            # bolt_prim = self._get_bolt_prim()  # MOD
            # bolt_pose = self._get_bolt_world_position()  # MOD
            # if bolt_pose is None or bolt_prim is None:
            #     return
            # bolt_pose[2] -= 0.05
            # if bolt_prim.IsValid():
            #     new_pos = Gf.Vec3d(float(x1), float(y1), float(bolt_pose[2]))
            #     bolt_prim.GetAttribute("xformOp:translate").Set(new_pos)
            
            # # 또 placing position으로 이동 (불필요하면 삭제하기)
            # action = self._apply_rmp_move(self._placing_position, euler_angles_to_quat(np.array([0, np.pi/2, 0])))


=======
        elif self.task_phase == 8:
            self._set_bolt_visibility(True)

            
            self.stage = omni.usd.get_context().get_stage()
            x1,y1,z1=self.robot.gripper.get_world_pose()[0]  # 함수 실행될 때마다 그리퍼 위치 가져오기
            target_prim = self.stage.GetPrimAtPath(self.target_path)
            target_pose = self.target.get_world_poses()[0][0]
            target_pose[2] -= 0.05
            if target_prim.IsValid():
                new_pos = Gf.Vec3d(float(x1), float(y1), float(target_pose[2]))
                target_prim.GetAttribute("xformOp:translate").Set(new_pos)
            
            nn=0
>>>>>>> feature/safety-system
            self.task_phase = 9

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

<<<<<<< HEAD
    # fixed joint 생성하는 함수
    def _create_fixed_joint(self):
        # stage = omni.usd.get_context().get_stage()
        if not self.bolt_prim_path:
            return
        joint_path = f"{self.bolt_prim_path}/MyFixedJoint"  # MOD: joint under selected bolt
        usd_joint = UsdPhysics.FixedJoint.Define(self.stage, Sdf.Path(joint_path))
        usd_joint.CreateBody0Rel().SetTargets([Sdf.Path("/World/UR10/ee_link/gripper_tip")])
        usd_joint.CreateBody1Rel().SetTargets([Sdf.Path(self.bolt_prim_path)])  # MOD
        usd_joint.CreateJointEnabledAttr(True)
        get_physx_interface().force_load_physics_from_usd()  # 물리 엔진에 즉시 반영
        self.joint_created = True
        print("Fixed Joint Created")

    # fixed joint 제거하는 함수
    def _remove_fixed_joint(self):
        if not self.bolt_prim_path:
            return
        joint_path = f"{self.bolt_prim_path}/MyFixedJoint"  # MOD
        omni.kit.commands.execute("DeletePrims", paths=[joint_path])  # MOD
        get_physx_interface().force_load_physics_from_usd()
        self.joint_created = False
        print("Fixed Joint Removed")
=======
>>>>>>> feature/safety-system

    def _sync_bolt_to_gripper(self):
        # 볼트를 실시간으로 이동하는 코드
        # self.stage = omni.usd.get_context().get_stage()
        x1,y1,z1=self.robot.gripper.get_world_pose()[0]  # 함수 실행될 때마다 그리퍼 위치 가져오기
<<<<<<< HEAD
        bolt_prim = self._get_bolt_prim()  # MOD
        if bolt_prim.IsValid():
            new_pos = Gf.Vec3d(float(x1), float(y1), float(z1-0.17))
            bolt_prim.GetAttribute("xformOp:translate").Set(new_pos)
            fixed_quat = euler_angles_to_quat(np.array([-np.pi/2, 0, 0]))
            orient_attr = None
            if bolt_prim.HasAttribute("xformOp:orient"):
                orient_attr = bolt_prim.GetAttribute("xformOp:orient")
            elif bolt_prim.HasAttribute("xformOp:orientation"):
                orient_attr = bolt_prim.GetAttribute("xformOp:orientation")
            if orient_attr is not None:
                if orient_attr.GetTypeName() == Sdf.ValueTypeNames.Quatf:
                    new_ori = Gf.Quatf(
                        float(fixed_quat[0]),
                        float(fixed_quat[1]),
                        float(fixed_quat[2]),
                        float(fixed_quat[3]),
                    )
                else:
                    new_ori = Gf.Quatd(
                        float(fixed_quat[0]),
                        float(fixed_quat[1]),
                        float(fixed_quat[2]),
                        float(fixed_quat[3]),
                    )
                orient_attr.Set(new_ori)

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
=======
        target_prim = self.stage.GetPrimAtPath(self.target_path)
        if target_prim.IsValid():
            if "bolt" in self.target.name.lower(): 
                new_pos = Gf.Vec3d(float(x1), float(y1-0.005), float(z1-0.17))
                fixed_quat = euler_angles_to_quat(np.array([-np.pi/2, 0, 0]))
                new_ori = Gf.Quatd(float(fixed_quat[0]), float(fixed_quat[1]), 
                                float(fixed_quat[2]), float(fixed_quat[3]))
            else:
                new_pos = Gf.Vec3d(float(x1), float(y1), float(z1-0.13))
                fixed_quat = euler_angles_to_quat(np.array([0, 0, 0]))
                new_ori = Gf.Quatd(float(fixed_quat[0]), float(fixed_quat[1]), 
                                float(fixed_quat[2]), float(fixed_quat[3]))
            # new_pos = Gf.Vec3d(float(x1), float(y1), float(z1-0.17))
            target_prim.GetAttribute("xformOp:translate").Set(new_pos)
            if target_prim.HasAttribute("xformOp:orient"):
                target_prim.GetAttribute("xformOp:orient").Set(new_ori)
            elif target_prim.HasAttribute("xformOp:orientation"):
                target_prim.GetAttribute("xformOp:orientation").Set(new_ori)
            

    def _set_bolt_visibility(self, visible: bool):
        """볼트의 가시성을 설정함 (True: 보임, False: 숨김)!"""
        self.stage = omni.usd.get_context().get_stage()
        target_prim = self.stage.GetPrimAtPath(self.target_path)
        
        if target_prim.IsValid():
            # 'visibility' 속성을 가져오거나 생성함!
            vis_attr = target_prim.GetAttribute("visibility")
            if not vis_attr:
                vis_attr = target_prim.CreateAttribute("visibility", Sdf.ValueTypeNames.Token)
            
            # True면 'inherited', False면 'invisible' 설정!
            new_vis = "inherited" if visible else "invisible"
            vis_attr.Set(new_vis)
>>>>>>> feature/safety-system
