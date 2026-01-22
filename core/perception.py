from omni.isaac.kit import SimulationApp

# # 1. 시뮬레이션 앱 실행 (가장 먼저 호출되어야 함)
# simulation_app = SimulationApp({"headless": False})

# from omni.isaac.sensor import Camera
from isaacsim.sensors.camera import Camera as IsaacCamera
from isaacsim.core.api import World

class Camera:
    def __init__(self, camera_prim_path="/World/UR10/ee_link/Camera", resolution=(640, 480), frequency=30):
        import omni

        self.world = World(stage_units_in_meters=1.0)
        self._initialized = False
        
        self.camera_path = camera_prim_path
        stage = omni.usd.get_context().get_stage()
        if stage is not None and not stage.GetPrimAtPath(self.camera_path).IsValid():
            print(f"Warning: camera prim not found at {self.camera_path}, creating one.")
        self.camera = IsaacCamera(
            prim_path=self.camera_path,
            frequency=frequency,
            resolution=resolution,
        )
        
    def setup_scene(self):
        # UR10 끝단 카메라를 사용하므로 씬 변경 없음.
        return
    
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
