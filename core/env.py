# core/env.py

from omni.isaac.kit import SimulationApp
from pxr import Gf, Usd, Sdf  # Sdf 추가

from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.manipulators.grippers import SurfaceGripper
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.core.api.world import World
import numpy as np


class EnvManager:
    def __init__(self, usd_path):
        import omni
        # stage를 가져올 때 get_stage()가 None을 반환할 수 있으므로 
        # 오픈 후에 stage를 다시 할당하는 것이 더 안전합니다.
        self.usd_path = usd_path
        self._world = None
        self._load_world()
        self.stage = omni.usd.get_context().get_stage()

    def _load_world(self):
        import omni.usd
        import os
        if os.path.exists(self.usd_path):
            omni.usd.get_context().open_stage(self.usd_path)
        else:
            raise FileNotFoundError(f"USD file not found at {self.usd_path}")

        self._world = World(stage_units_in_meters=1.0)
        
        assets_root_path = get_assets_root_path()
        asset_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
        bolt_asset_path = "/home/rokey/Desktop/DTHRC/DTHRC/assets/factory_bolt_m20_loose.usd"
        robot = add_reference_to_stage(usd_path=asset_path, prim_path="/World/UR10")
        add_reference_to_stage(usd_path=bolt_asset_path, prim_path="/World/Bolt")
        robot.GetVariantSet("Gripper").SetVariantSelection("Short_Suction")
        gripper = SurfaceGripper(
            end_effector_prim_path="/World/UR10/ee_link", surface_gripper_path="/World/UR10/ee_link/SurfaceGripper"
        )
        self.robot_position = np.array([-0.55, 0.8, 1.0])
        ur10 = self._world.scene.add(
            SingleManipulator(
                prim_path="/World/UR10", name="my_ur10", end_effector_prim_path="/World/UR10/ee_link", gripper=gripper, position = self.robot_position
            )
        )
        ur10.set_default_state(position = self.robot_position)
        ur10.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))


    def setup_physics(self):
        import omni.kit.commands
        # '/World/PhysicsScene' 문자열을 Sdf.Path로 변환
        physics_path = Sdf.Path('/World/PhysicsScene')
        
        if not self.stage.GetPrimAtPath(physics_path):
            omni.kit.commands.execute('AddPhysicsSceneCommand', 
                                      stage=self.stage, 
                                      path='/World/PhysicsScene')
