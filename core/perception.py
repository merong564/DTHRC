from omni.isaac.kit import SimulationApp

# # 1. 시뮬레이션 앱 실행 (가장 먼저 호출되어야 함)
# simulation_app = SimulationApp({"headless": False})

# from omni.isaac.sensor import Camera
from isaacsim.sensors.camera import Camera
from isaacsim.core.api import World
import omni.isaac.core.utils.prims as prim_utils
import omni.isaac.core.utils.rotations as rot_utils
import numpy as np

class Camera:
    def __init__(self):
        from isaacsim.sensors.camera import Camera
        from isaacsim.core.api import World
        
        self.world = World(stage_units_in_meters=1.0)
        self._initialized = False
        
        # 2. 카메라 생성 위치 및 경로 설정
        self.camera_path = "/World/Camera"
        self.camera = Camera(
            prim_path=self.camera_path,
            position=np.array([1.0, 2.7, 2.7]), # 로봇이나 작업대 앞 위치
            frequency=30,
            resolution=(640, 480),
            orientation=rot_utils.euler_angles_to_quat(np.array([-90, 90, 0]), degrees=True) # 아래를 내려다보도록 회전
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