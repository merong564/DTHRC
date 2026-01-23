from omni.isaac.kit import SimulationApp

# # 1. 시뮬레이션 앱 실행 (가장 먼저 호출되어야 함)
# simulation_app = SimulationApp({"headless": False})

# from omni.isaac.sensor import Camera
from isaacsim.sensors.camera import Camera
from isaacsim.core.api import World
import omni.isaac.core.utils.prims as prim_utils
import omni.isaac.core.utils.rotations as rot_utils
import numpy as np
from pxr import Gf

class Camera:
    def __init__(self):
        from isaacsim.sensors.camera import Camera
        from isaacsim.core.api import World
        
        self.world = World(stage_units_in_meters=1.0)
        self._initialized = False
        
        # 2. 카메라 생성 위치 및 경로 설정
        self.camera_path = "/World/Camera"
        base_orientation = rot_utils.euler_angles_to_quat(np.array([-90, 90, 0]), degrees=True)
        world_x_rot = Gf.Rotation(Gf.Vec3d(1, 0, 0), 30).GetQuat()
        base_quat = Gf.Quatd(
            float(base_orientation[0]),
            float(base_orientation[1]),
            float(base_orientation[2]),
            float(base_orientation[3]),
        )
        target_quat = world_x_rot * base_quat
        target_orientation = np.array([target_quat.GetReal(), *target_quat.GetImaginary()])

        self.camera = Camera(
            prim_path=self.camera_path,
            position=np.array([0.8916, -1.4017, 3.139]), # 로봇이나 작업대 앞 위치
            frequency=30,
            resolution=(640, 480),
            orientation=target_orientation,
        )
        
    def setup_scene(self):
        # 테스트용 볼트/너트가 놓일 바닥과 조명 추가
        self.world.scene.add_default_ground_plane()
        # 카메라 초기화
        # self.camera.initialize()
    
    def initialize(self):
        """외부(main.py)에서 명시적으로 호출"""
        if self._initialized:
            return
        self.camera.initialize()
        self._initialized = True

        # # 각도 조정
        # target_quat = rot_utils.euler_angles_to_quat(np.array([30, 0, 0]), degrees=True)

        # xforms_utils.set_world_pose(
        #     prim_path=self.camera_path,
        #     translation=np.array([1.15, 1.6, 3.5]),
        #     orientation=target_quat
        # )

    def step(self):
        """
        ✔ 시뮬레이션 1 step
        ✔ 카메라 이미지 1장 반환
        """
        self.world.step(render=True)

        if self.world.is_playing():
            rgba = self.camera.get_rgba()
            return rgba

        return None
    
    

if __name__ == "__main__":
    tester = Camera()
    tester.setup_scene()
    tester.run()
