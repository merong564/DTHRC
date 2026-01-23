from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})


import numpy as np
import sys
import carb
import asyncio
import omni

from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import SurfaceGripper
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.api.world import World
from isaacsim.core.prims import GeometryPrim, RigidPrim
from isaacsim.core.utils.rotations import euler_angles_to_quat
import isaacsim.robot_motion.motion_generation as mg
from isaacsim.core.prims import SingleArticulation
import omni.kit.commands
from pxr import Usd, UsdGeom, Gf, UsdPhysics, Sdf, UsdLux, PhysxSchema

# from isaacsim.robot.manipulators.examples.universal_robots.controllers.pick_place_controller import PickPlaceController

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


class Tutorial_UR10:
    def __init__(self) -> None:
        self.usd_path = "/home/rokey/Desktop/DTHRC/DTHRC/assets/env_gripper.usd"
        self._world = World(stage_units_in_meters=1.0)
        self._placing_position = np.array([-1.25, -0.25047, 1.5])
        self._end_effector_offset = np.array([0, 0, 0.02])
        self._task_done = False
        self.joint_created = False
        
        self.my_controller = None
        self.articulation_controller = None
        self.cube = None
        self.robots = None
        return

    
    def setup_scene(self):
        import os
        import omni.usd
        if os.path.exists(self.usd_path):
            omni.usd.get_context().open_stage(self.usd_path)
        else:
            raise FileNotFoundError(f"USD file not found at {self.usd_path}")
        self._world = World(stage_units_in_meters=1.0, physics_dt=1/200, rendering_dt=20/200)
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            sys.exit()

        asset_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
        # bolt_asset_path = assets_root_path + "/Isaac/Props/Factory/factory_bolt_m20_loose.usd"
        bolt_asset_path = "/home/rokey/Desktop/DTHRC/DTHRC/assets/factory_bolt_m20_loose.usd"
        robot = add_reference_to_stage(usd_path=asset_path, prim_path="/World/UR10")
        add_reference_to_stage(usd_path=bolt_asset_path, prim_path="/World/Bolt")
        robot.GetVariantSet("Gripper").SetVariantSelection("Short_Suction")
        gripper = SurfaceGripper(
            end_effector_prim_path="/World/UR10/ee_link", surface_gripper_path="/World/UR10/ee_link/SurfaceGripper"
        )
        self.robot_position = np.array([0.0, -0.54194, 1.0])
        ur10 = self._world.scene.add(
            SingleManipulator(
                prim_path="/World/UR10", name="my_ur10", end_effector_prim_path="/World/UR10/ee_link", gripper=gripper, position = self.robot_position
            )
        )
        
        ur10.set_default_state(position = self.robot_position)
        ur10.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        self._world.scene.add(
            RigidPrim(
                prim_paths_expr="/World/Bolt",
                name = "my_bolt",
                positions = np.array([[0.7931, -0.36331, 0.88053]]),
                scales = np.array([[2, 2, 2]]),
                orientations = np.array([euler_angles_to_quat(np.array([-np.pi/2, 0, 0]))])
            )
        )
        return

    def setup_post_load(self):
        self._world.reset()
        self.robots = self._world.scene.get_object("my_ur10")
        self.bolt = self._world.scene.get_object("my_bolt")
        
        self.my_controller = RMPFlowController(
            name="my_ur10_cspace_controller", 
            robot_articulation=self.robots,
            attach_gripper = True
        )
        self.articulation_controller = self.robots.get_articulation_controller()
        self.task_phase = 1

    def physics_step(self,):
        import omni
        from pxr import Sdf, UsdPhysics

        if self.task_phase == 1:
            self.bolt_position = self.bolt.get_world_poses()[0][0]
            self.bolt_position[2] += 0.035
            # print(self.bolt_position)
            if self.bolt_position[0] >= 0.5:
                print("close bolt")
                self.task_phase = 2

        #컨베이어 추가 시 수정 필요
        elif self.task_phase == 2:
            # print(f"task_phase :{self.task_phase} conveyor")
            x,y,z = self.bolt.get_world_poses()[0][0]
            self.task_phase = 3

        elif self.task_phase == 3:
            # print(f"task_phase :{self.task_phase} bolt 0.4 access")
            x,y,z = self.bolt.get_world_poses()[0][0]
            self.bolt_position1 = np.array([x,y,z])
            target_position = self.bolt_position1
            # target_position[2] = self.bolt_position[2]+0.04
            # print(f"#############{target_position}#####################")

            end_effector_orientation = euler_angles_to_quat(np.array([0, np.pi/2, 0]))
            action = self.my_controller.forward(
                target_end_effector_position=target_position, 
                target_end_effector_orientation=end_effector_orientation
            )
            self.robots.apply_action(action)
            ee_pose = self.robots.gripper.get_world_pose()[0] # 그리퍼(ee) 위치
            bolt_pose = self.bolt.get_world_poses()[0][0]    # 볼트 위치
            distance = np.linalg.norm(ee_pose - bolt_pose)
            print(distance)
            if distance < 0.25 and not self.joint_created:
                print(f"Distance: {distance:.4f}m - Creating Fixed Joint!")
                
                #돌아간느 코드
                stage = omni.usd.get_context().get_stage()
                joint_path = "/World/UR10/ee_link/MyFixedJoint"
                usd_joint = UsdPhysics.FixedJoint.Define(stage, Sdf.Path(joint_path))
                usd_joint.CreateBody0Rel().SetTargets([Sdf.Path("/World/UR10/ee_link")])
                usd_joint.CreateBody1Rel().SetTargets([Sdf.Path("/World/Bolt")])
                usd_joint.CreateJointEnabledAttr(True)


                # --- [추가] 물리 엔진에 즉시 반영 ---
                from omni.physx import get_physx_interface
                get_physx_interface().force_load_physics_from_usd()
                # ----------------------------------
                self.joint_created = True
                self.task_phase = 6
                print(f"Fixed Joint Created: {joint_path}")
            current_joint_positions = self.robots.get_joint_positions()
            
            
            if np.all(np.abs(current_joint_positions[:6] - action.joint_positions) < 0.001):
                self.my_controller.reset()
                self.bolt_position[2] += 0.4
                
    
        # elif self.task_phase == 4:
        #     print(f"task_phase :{self.task_phase} bolt z down")
        #     target_position = self.bolt_position

        #     end_effector_orientation = euler_angles_to_quat(np.array([0, np.pi/2, 0]))
        #     action = self.my_controller.forward(
        #         target_end_effector_position=target_position, 
        #         target_end_effector_orientation=end_effector_orientation
        #     )
        #     self.robots.apply_action(action)
        #     current_joint_positions = self.robots.get_joint_positions()
            

        #     if np.all(np.abs(current_joint_positions[:6] - action.joint_positions) < 0.001):
        #         self.my_controller.reset()
        #         self.task_phase = 5
        
        # elif self.task_phase == 5:
        #     print(f"task_phase :{self.task_phase} gripper close")
        #     self.robots.gripper.close()
        #     self.task_phase = 6
        
        elif self.task_phase == 6:
            print(f"task_phase :{self.task_phase} bolt z up")
            x,y,z = self.bolt.get_world_poses()[0][0]
            x1,y1,z1=self.robots.gripper.get_world_pose()[0]

            ############33 돌아가는 코드
            stage1 = omni.usd.get_context().get_stage()
            target_position= np.array([x,y,self.bolt_position[2]])
            x1,y1,z1=self.robots.gripper.get_world_pose()[0]

            self.bolt_prim = stage1.GetPrimAtPath("/World/Bolt")
            if self.bolt_prim.IsValid():
                new_pos = Gf.Vec3d(float(x1), float(y1), float(z1-0.17))
                self.bolt_prim.GetAttribute("xformOp:translate").Set(new_pos)
            #################3
            
            end_effector_orientation = euler_angles_to_quat(np.array([0, np.pi/2, 0]))
            action = self.my_controller.forward(
                target_end_effector_position=np.array([x, y, z1+0.05]),
                target_end_effector_orientation=end_effector_orientation
            )
            self.robots.apply_action(action)
            if z1> 1.5:
                self.task_phase = 7




            current_joint_positions = self.robots.get_joint_positions()
            
            if np.all(np.abs(current_joint_positions[:6] - action.joint_positions) < 0.001):
                self.my_controller.reset()
                self.task_phase = 7
        
        elif self.task_phase == 7:
            print(f"task_phase :{self.task_phase} place")
            placing_position = self._placing_position
            x,y,z = self.bolt.get_world_poses()[0][0]
            x1,y1,z1=self.robots.gripper.get_world_pose()[0]

            ############33
            stage1 = omni.usd.get_context().get_stage()
            target_position= np.array([x,y,self.bolt_position[2]])
            x1,y1,z1=self.robots.gripper.get_world_pose()[0]

            self.bolt_prim = stage1.GetPrimAtPath("/World/Bolt")
            if self.bolt_prim.IsValid():
                new_pos = Gf.Vec3d(float(x1), float(y1), float(z1-0.17))
                self.bolt_prim.GetAttribute("xformOp:translate").Set(new_pos)
            #################3



            end_effector_orientation = euler_angles_to_quat(np.array([0, np.pi/2, 0]))
            action = self.my_controller.forward(
                target_end_effector_position=placing_position, 
                target_end_effector_orientation=end_effector_orientation
            )
            self.robots.apply_action(action)
            current_joint_positions = self.robots.get_joint_positions()
            
            if np.all(np.abs(current_joint_positions[:6] - action.joint_positions) < 0.001):
                self.my_controller.reset()
                self.task_phase = 9
        
        # elif self.task_phase == 8:
        #     print(f"task_phase :{self.task_phase} gripper open")
        #     self.robots.gripper.open()
        #     self.task_phase = 9
        
        elif self.task_phase == 9:
            print(f"task_phase :{self.task_phase} finish up")
            joint_path = "/World/UR10/ee_link/MyFixedJoint"

            # if self.joint_created:
            #     omni.kit.commands.execute(
            #         "DeletePrims",
            #         paths=[joint_path]
            #     )
            #     self.joint_created = False # 플래그 초기화 (재사용 가능하도록)
            #     print(f"Fixed Joint at {joint_path} removed.")

            
            if self.joint_created:
                # 2. omni.kit.commands를 사용하여 USD에서 조인트 프림 삭제
                import omni.kit.commands
                omni.kit.commands.execute(
                    "DeletePrims",
                    paths=[joint_path]
                )
                
                # 3. 중요: 물리 엔진에 삭제 상태를 즉시 동기화
                from omni.physx import get_physx_interface
                get_physx_interface().force_load_physics_from_usd()
                
                # 4. 상태 초기화
                self.joint_created = False
                print(f"Fixed Joint at {joint_path} removed and physics reloaded.")

            # 이후 이동 로직 (다음 페이즈로 전환 등)
            target_position = self._placing_position


            end_effector_orientation = euler_angles_to_quat(np.array([0, np.pi/2, 0]))
            action = self.my_controller.forward(
                target_end_effector_position=target_position, 
                target_end_effector_orientation=end_effector_orientation
            )
            self.robots.apply_action(action)
            current_joint_positions = self.robots.get_joint_positions()
            
            if np.all(np.abs(current_joint_positions[:6] - action.joint_positions) < 0.001):
                self.my_controller.reset()
                self.task_phase = 10
        
        return


            
        # print(self.bolt.get_world_poses()[0][0])
        # picking_position = self.bolt.get_world_poses()[0][0]
        # x,y,z = self.bolt.get_world_poses()[0][0]
        # print(z)
        # picking_position = np.array([x, y, z+1])
        # actions = self.my_controller.forward(
        #     picking_position=picking_position,
        #     placing_position=np.array([-1.25, -0.25047, 1.0]),
        #     current_joint_positions=self.robots.get_joint_positions(),
        #     end_effector_offset=self._end_effector_offset,
        # )

        # self.articulation_controller.apply_action(actions)

        # if self.my_controller.is_done():
        #     carb.log_info("Pick and Place �묒뾽 �꾨즺")
        #     self._task_done = True
        # return

    async def setup_pre_reset(self):
        if self.my_controller is not None:
            self.my_controller.reset()
        self._task_done = False
        return
    
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    frame_count = 0

if __name__=="__main__":
    tutorial = Tutorial_UR10()
    tutorial.setup_scene()
    tutorial.setup_post_load()

    # asyncio.get_event_loop().run_until_complete(tutorial.setup_post_load())
    while simulation_app.is_running():
        tutorial._world.step(render=True)

        if tutorial._world.is_playing():
            # if tutorial._world.current_time_step_index == 0:
            #     tutorial._world.reset()
            #     tutorial.my_controller.reset()
            tutorial.physics_step()


    simulation_app.close()

    