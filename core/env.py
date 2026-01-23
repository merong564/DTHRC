from omni.isaac.kit import SimulationApp
from pxr import Gf, Usd, Sdf
import numpy as np
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import SurfaceGripper
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.rotations import euler_angles_to_quat

from isaacsim.core.prims import GeometryPrim, RigidPrim

class EnvManager:
    def __init__(self, usd_path=None, robot_usd_path=None):
        from omni.isaac.core import World
        import omni.usd
        
        self.usd_path = usd_path
        self.robot_usd_path = robot_usd_path

        if self.usd_path:
            self._load_world()

        self.world = World(stage_units_in_meters=1.0, physics_dt=1/500, rendering_dt=20/200)
        self.stage = omni.usd.get_context().get_stage()
        
    def _load_world(self):
        import os
        import omni.usd
        if os.path.exists(self.usd_path):
            omni.usd.get_context().open_stage(self.usd_path)
        else:
            raise FileNotFoundError(f"USD file not found at {self.usd_path}")
        
    

    def add_robot(self, robot_usd_path, prim_path="/World/UR10", name="my_ur10", position=np.array([0.0, -0.54194, 1.0])):
        """
        특정 USD 경로에서 로봇을 불러와 환경에 추가합니다.
        """
        robot_usd_path1 = "/home/rokey/Desktop/DTHRC/DTHRC/assets/ur10/ur10.usd"
        # 1. Stage에 로봇 USD 참조 추가
        robot = add_reference_to_stage(usd_path=robot_usd_path, prim_path=prim_path)
        robot.GetVariantSet("Gripper").SetVariantSelection("Short_Suction")

        # 2. 로봇 위치 설정 (값이 있을 경우)
        if position is not None:
            self.robot_position = position

        gripper = SurfaceGripper(
            end_effector_prim_path="/World/UR10/ee_link", 
            surface_gripper_path="/World/UR10/ee_link/SurfaceGripper"
        )

        ur10 = self.world.scene.add(
            SingleManipulator(
                prim_path="/World/UR10", 
                name="my_ur10", 
                end_effector_prim_path="/World/UR10/ee_link", 
                gripper=gripper, 
                position =  self.robot_position
            ))
            
        ur10.set_default_state(position = self.robot_position)
        ur10.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        # 클래스 외부에서도 접근할 수 있도록 인스턴스 변수에 할당
        #self.robot = ur10
        self.robot = self.world.scene.get_object(ur10)

        print(f"Robot loaded at: {prim_path}")
        return self.robot
    
    def add_bolt(self, bolt_usd_path, position=np.array([0.7931, -0.36331, 0.88053])):
        add_reference_to_stage(usd_path=bolt_usd_path, prim_path="/World/Bolt")
        

        bolt = self.world.scene.add(
            RigidPrim(
                prim_paths_expr="/World/Bolt",
                name = "my_bolt",
                positions = np.array([[0.7931, -0.36331, 0.88053]]),
                scales = np.array([[2, 2, 2]]),
                orientations = np.array([euler_angles_to_quat(np.array([-np.pi/2, 0, 0]))])
            )
        )
        self.bolt = bolt
        return bolt
