import numpy as np
import math
from pxr import UsdGeom, Gf, Usd, UsdShade
from omni.isaac.core.utils.semantics import add_update_semantics

class SafetyManager:
    def __init__(self, stage, lidar_interface):
        self.stage = stage
        self.lidar_interface = lidar_interface
        self.robot_path = "/World/UR10/base_link"
        self.lidar_full_path = f"{self.robot_path}/LidarName"
        self.human_path = "/World/male"
        self.danger_path = "/World/danger"
        
        self.human_prim = stage.GetPrimAtPath(self.human_path)
        self.danger_prim = stage.GetPrimAtPath(self.danger_path)
        
        self._setup_lidar()
        self._setup_semantics()

    def _setup_lidar(self):
        import omni.kit.commands
        omni.kit.commands.execute("RangeSensorCreateLidar",    
            path="/LidarName", parent=self.robot_path, 
            min_range=0.4, max_range=20.0, draw_points=True, 
            horizontal_fov=360.0, vertical_fov=60.0, enable_semantics=True, 
            rotation_rate=0, horizontal_resolution=1.0, vertical_resolution=1.0
        )

    def _setup_semantics(self):
        if self.human_prim.IsValid():
            for prim in Usd.PrimRange(self.human_prim):
                if prim.IsA(UsdGeom.Mesh):
                    add_update_semantics(prim=prim, semantic_label="human")

    def change_led_color(self, r, g, b, intensity=10000.0):
        if self.danger_prim.IsValid():
            mat_binding = UsdShade.MaterialBindingAPI(self.danger_prim)
            material_rel = mat_binding.GetDirectBinding().GetMaterial()
            if material_rel:
                shader_prim = self.stage.GetPrimAtPath(f"{material_rel.GetPath()}/Shader")
                if shader_prim.IsValid():
                    shader_prim.GetAttribute("inputs:emissive_color").Set(Gf.Vec3f(r, g, b))
                    shader_prim.GetAttribute("inputs:emissive_intensity").Set(intensity)

    def get_human_distance(self):
        semantics = self.lidar_interface.get_prim_data(self.lidar_full_path)
        depth = self.lidar_interface.get_linear_depth_data(self.lidar_full_path)
        
        if len(semantics) > 0:
            semantics_np = np.array(semantics)
            depth_np = np.array(depth)
            human_indices = np.where(semantics_np == self.human_path)[0]
            if len(human_indices) > 0:
                return np.min(depth_np[human_indices])
        return None