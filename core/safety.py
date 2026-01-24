import numpy as np
import math
from pxr import Gf, UsdShade

class SafetyManager:
    def __init__(
        self,
        stage,
        lidar_interface,
        robot_path="/World/UR10/base_link",
        lidar_name="LidarName",
        human_path="/World/male",
        danger_path="/World/danger",
    ):
        self.stage = stage
        self.lidar_interface = lidar_interface
        self.robot_path = robot_path
        self.lidar_full_path = f"{self.robot_path}/{lidar_name}"
        self.human_path = human_path
        self.danger_path = danger_path
        
        self.human_prim = stage.GetPrimAtPath(self.human_path)
        self.danger_prim = stage.GetPrimAtPath(self.danger_path)
        
        return

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

    def update_led_for_distance(self, dist):
        if dist is not None:
            if dist < 1.4:
                self.change_led_color(1.0, 0.0, 0.0)
            elif dist < 2.2:
                self.change_led_color(1.0, 1.0, 0.0)
            else:
                self.change_led_color(0.0, 0.0, 1.0)
        else:
            self.change_led_color(0.0, 0.0, 1.0)
