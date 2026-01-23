import numpy as np
from omni.isaac.core.utils.rotations import euler_angles_to_quat
import isaacsim.robot_motion.motion_generation as mg
from pxr import Sdf, UsdPhysics, Gf
import omni.kit.commands
from omni.physx import get_physx_interface
from isaacsim.core.prims import SingleArticulation

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
    # def __init__(self, world, robot, bolt, placing_position):
    def __init__(self, world, robot,placing_position):
        self.world = world
        self.robot =self.world.scene.get_object("my_ur10")
        # self.bolt = bolt
        self.bolt = self.world.scene.get_object("my_bolt")
        self.my_controller = None
        self.articulation_controller = None
        print("bolt goood")
        self.stage = world.stage
        
        # RMPFlow 설정
        # config = mg.interface_config_loader.load_supported_motion_policy_config("UR10", "RMPflowSuction")
        # self.rmp_flow = mg.lula.motion_policies.RmpFlow(**config)
        # self.physics_dt = 1/200
        # self.articulation_rmp = mg.ArticulationMotionPolicy(self.robot, self.rmp_flow, self.physics_dt) # 원래는 SingleArticulation, 이를 self.robot으로 바꿈
        # from isaacsim.storage.native import get_assets_root_path
        # from isaacsim.core.utils.stage import add_reference_to_stage
        # from isaacsim.robot.manipulators import SingleManipulator
        # from isaacsim.robot.manipulators.grippers import SurfaceGripper

        # assets_root_path = get_assets_root_path()
        # asset_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
        # # bolt_asset_path = assets_root_path + "/Isaac/Props/Factory/factory_bolt_m20_loose.usd"
        # robot = add_reference_to_stage(usd_path=asset_path, prim_path="/World/UR10")
        # robot.GetVariantSet("Gripper").SetVariantSelection("Short_Suction")
        # gripper = SurfaceGripper(
        #     end_effector_prim_path="/World/UR10/ee_link", 
        #     surface_gripper_path="/World/UR10/ee_link/SurfaceGripper"
        # )
        # self.robot_position = np.array([0.0, -0.54194, 1.0])
        # ur10 = self.world.scene.add(
        #     SingleManipulator(
        #         prim_path="/World/UR10", 
        #         name="my_ur10", 
        #         end_effector_prim_path="/World/UR10/ee_link", 
        #         gripper=gripper, 
        #         position = self.robot_position
        #     )
        # )
        
        # ur10.set_default_state(position = self.robot_position)
        # ur10.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))





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

    def control_robot(self):
        # 1. 현재 정보 업데이트
        print("control_robot")
        ee_pose, _ = self.robot.gripper.get_world_pose()
        bolt_pose, _ = self.bolt.get_world_poses()
        bolt_pose = bolt_pose[0]
        print("control_robot2222")
        print(bolt_pose)
        
        # 2. 페이즈별 로직 (State Machine)
        if self.task_phase == 1: # 볼트 접근 감시
            bolt_pose[2] += 0.035    # 볼트보다 0.035 높은 위치를 잡음
            print("control_robot3333333")
            if bolt_pose[0] >= 0.5:           # 0.5: 로봇이 pick하기 시작하는 시점, 튜닝 필요
                print("close bolt")
                self.task_phase = 3

        # pick 대기 시간 주기 (컨베이어 추가 시 수정 필요)
        elif self.task_phase == 2:
            # print(f"task_phase :{self.task_phase} conveyor")
            # x,y,z = self.bolt.get_world_poses()[0][0]
            self.task_phase = 3

        elif self.task_phase == 3: # 볼트로 이동
            print(f"task_phase :{self.task_phase} bolt access")
            # target_pos = bolt_pose.copy()
            target_pos = self.bolt.get_world_poses()[0][0]       # 매순간 볼트의 위치를 가져와 타겟으로 설정

            target_ori = euler_angles_to_quat(np.array([0, np.pi/2, 0]))    # 그리퍼가 접근하는 각도
            
            action = self._apply_rmp_move(target_pos, target_ori)    # 로봇이 타겟으로 이동
            
            dist = np.linalg.norm(ee_pose - bolt_pose)
            print(dist)

            # 엔드 이펙터 위치와 볼트 위치가 가까워지면 fixed joint 생성, 다음 페이즈로 이동
            if dist < 0.25 and not self.joint_created:  # 0.25 튜닝 필요
                print(f"Distance: {dist:.4f}m - Creating Fixed Joint!")
                self._create_fixed_joint()
                self.task_phase = 6

        elif self.task_phase == 6: # 들어올리기
            # 매순간 볼트, 엔드 이펙터 위치 가져오기
            bolt_pose = self.bolt.get_world_poses()[0][0]
            ee_pose = self.robot.gripper.get_world_pose()[0]

            print(f"task_phase :{self.task_phase} picking: bolt z up")
            self._sync_bolt_to_gripper()  # 볼트 위치 이동
            target_pos = np.array([bolt_pose[0], bolt_pose[1], bolt_pose[2]+0.05])
            ## 추후 수정해보기: 로봇팔 움직인 후에 볼트 위치 변경
            action = self._apply_rmp_move(target_pos, euler_angles_to_quat(np.array([0, np.pi/2, 0])))
            
            if ee_pose[2] > 1.5:   # 로봇팔 위치가 1.5를 넘으면 다음 페이즈로 이동
                self.task_phase = 7
            
            ## 원래 코드
            current_joint_positions = self.robot.get_joint_positions()
            
            if np.all(np.abs(current_joint_positions[:6] - action.joint_positions) < 0.001):
                self.my_controller.reset()
                self.task_phase = 7
        
        elif self.task_phase == 7: # 목표 지점으로 이동
            print(f"task_phase :{self.task_phase} placing")
            ee_pose = self.bolt.get_world_poses()[0][0]
            self._sync_bolt_to_gripper()
            action = self._apply_rmp_move(self._placing_position, euler_angles_to_quat(np.array([0, np.pi/2, 0])))
            current_joint_positions = self.robot.get_joint_positions()
            # 원래 페이즈 변경 코드
            if np.all(np.abs(current_joint_positions[:6] - action.joint_positions) < 0.001):
                self.my_controller.reset()
                self.task_phase = 8

            if np.linalg.norm(ee_pose - self._placing_position) < 0.05:
                self.task_phase = 8

        elif self.task_phase == 8: # 조인트 해제 및 종료
            print(f"task_phase :{self.task_phase} finish up")
            if self.joint_created:
                self._remove_fixed_joint()
            
            # 또 placing position으로 이동 (불필요하면 삭제하기)
            action = self._apply_rmp_move(self._placing_position, euler_angles_to_quat(np.array([0, np.pi/2, 0])))


            self.task_phase = 9

    # 로봇이 target으로 이동하는 함수
    def _apply_rmp_move(self, pos, ori):
        action = self.my_controller.forward(
            target_end_effector_position=pos,
            target_end_effector_orientation=ori
        )
        self.robot.apply_action(action)
        return action

    # fixed joint 생성하는 함수
    def _create_fixed_joint(self):
        # stage = omni.usd.get_context().get_stage()
        joint_path = "/World/UR10/ee_link/MyFixedJoint"
        usd_joint = UsdPhysics.FixedJoint.Define(self.stage, Sdf.Path(joint_path))
        usd_joint.CreateBody0Rel().SetTargets([Sdf.Path("/World/UR10/ee_link")])
        usd_joint.CreateBody1Rel().SetTargets([Sdf.Path("/World/Bolt")])
        usd_joint.CreateJointEnabledAttr(True)
        get_physx_interface().force_load_physics_from_usd()  # 물리 엔진에 즉시 반영
        self.joint_created = True
        print("Fixed Joint Created")

    # fixed joint 제거하는 함수
    def _remove_fixed_joint(self):
        omni.kit.commands.execute("DeletePrims", paths=["/World/UR10/ee_link/MyFixedJoint"])
        get_physx_interface().force_load_physics_from_usd()
        self.joint_created = False
        print("Fixed Joint Removed")

    def _sync_bolt_to_gripper(self):
        # 볼트를 실시간으로 이동하는 코드
        self.stage = omni.usd.get_context().get_stage()
        x1,y1,z1=self.robot.gripper.get_world_pose()[0]  # 함수 실행될 때마다 그리퍼 위치 가져오기
        bolt_prim = self.stage.GetPrimAtPath("/World/Bolt")
        if bolt_prim.IsValid():
            new_pos = Gf.Vec3d(float(x1), float(y1), float(z1-0.17))
            bolt_prim.GetAttribute("xformOp:translate").Set(new_pos)