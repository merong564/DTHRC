# core/env.py

from omni.isaac.kit import SimulationApp
from pxr import Gf, Usd, Sdf  # Sdf 추가

class EnvManager:
    def __init__(self, usd_path):
        import omni
        # stage를 가져올 때 get_stage()가 None을 반환할 수 있으므로 
        # 오픈 후에 stage를 다시 할당하는 것이 더 안전합니다.
        self.usd_path = usd_path
        self._load_world()
        self.stage = omni.usd.get_context().get_stage()

    def _load_world(self):
        import omni.usd
        import os
        if os.path.exists(self.usd_path):
            omni.usd.get_context().open_stage(self.usd_path)
        else:
            raise FileNotFoundError(f"USD file not found at {self.usd_path}")

    def setup_physics(self):
        import omni.kit.commands
        # '/World/PhysicsScene' 문자열을 Sdf.Path로 변환
        physics_path = Sdf.Path('/World/PhysicsScene')
        
        if not self.stage.GetPrimAtPath(physics_path):
            omni.kit.commands.execute('AddPhysicsSceneCommand', 
                                      stage=self.stage, 
                                      path='/World/PhysicsScene')