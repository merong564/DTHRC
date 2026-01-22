from omni.isaac.kit import SimulationApp
from pxr import Gf, Usd, Sdf
import numpy as np
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import SurfaceGripper

class EnvManager:
    def __init__(self, usd_path=None, robot_usd_path=None):
        from omni.isaac.core import World
        import omni.usd
        
        self.usd_path = usd_path
        self.robot_usd_path = robot_usd_path

        if self.usd_path:
            self._load_world()

        # 1. World 초기화 (물리 및 렌더링 시간 설정)
        self.world = World(stage_units_in_meters=1.0, physics_dt=1/200, rendering_dt=20/200)
        
        self.stage = omni.usd.get_context().get_stage()
        # 3. Scene 초기화를 위해 반드시 reset() 수행
        #self.world.reset()

        self.world.reset()

        if self.robot_usd_path:
            self.add_robot(robot_usd_path)
        

        # self.robot = None

    def _load_world(self):
        import os
        import omni.usd
        if os.path.exists(self.usd_path):
            omni.usd.get_context().open_stage(self.usd_path)
        else:
            raise FileNotFoundError(f"USD file not found at {self.usd_path}")
        
    

    def add_robot(self, robot_usd_path, prim_path="/World/ur10", name="ur10", position=np.array([0.0, -0.54194, 0.8])):
        """
        특정 USD 경로에서 로봇을 불러와 환경에 추가합니다.
        """
        from omni.isaac.core.utils.stage import add_reference_to_stage
        
        # 1. Stage에 로봇 USD 참조 추가
        robot = add_reference_to_stage(usd_path=robot_usd_path, prim_path=prim_path)
        robot.GetVariantSet("Gripper").SetVariantSelection("Short_Suction")
        
        # 2. Robot 객체로 생성 (Articulation 기능 포함)
        # self.robot = Robot(prim_path=prim_path, name=name)

        gripper = SurfaceGripper(
            end_effector_prim_path="/World/ur10/ee_link",
            surface_gripper_path="/World/ur10/ee_link/SurfaceGripper"
        )
        ur10 = SingleManipulator(
                prim_path="/World/ur10", name="ur10", end_effector_prim_path="/World/ur10/ee_link", gripper = gripper)
            
        
        self.world.scene.add(ur10)
        
        ur10.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        
        # 3. 위치 설정 (값이 있을 경우)
        if position is not None:
            # self.robot.set_world_pose(position=np.array(position))
            ur10.set_world_pose(position=np.array(position))
            
        # # 4. World Scene에 로봇 등록
        # self.world.scene.add(self.robot)
        
        # print(f"Robot loaded at: {prim_path}")
        # return self.robot
        # 클래스 외부에서도 접근할 수 있도록 인스턴스 변수에 할당
        self.robot = ur10
        
        print(f"Robot loaded at: {prim_path}")
        return self.robot

    def setup_physics(self):
        # World 객체가 이미 물리 설정을 관리하므로, 필요시 추가 커스텀 설정만 수행합니다.
        pass