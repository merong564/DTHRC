import numpy as np
import omni.isaac.core.utils.rotations as rot_utils
from isaacsim.core.api import World
from isaacsim.sensors.camera import Camera as IsaacCamera
from omni.isaac.core.utils.semantics import add_update_semantics
from pxr import Gf, UsdGeom, Usd


class Camera:
    def __init__(self):
        self.world = World(stage_units_in_meters=1.0)
        self._initialized = False

        self.camera_path = "/World/Camera"
        base_orientation = rot_utils.euler_angles_to_quat(np.array([-90, 90, 0]), degrees=True)
        world_x_rot = Gf.Rotation(Gf.Vec3d(1, 0, 0), 40).GetQuat()              # 카메라를 45도 아래로 향하게 설정
        base_quat = Gf.Quatd(
            float(base_orientation[0]),
            float(base_orientation[1]),
            float(base_orientation[2]),
            float(base_orientation[3]),
        )
        target_quat = world_x_rot * base_quat
        target_orientation = np.array([target_quat.GetReal(), *target_quat.GetImaginary()])

        self.camera = IsaacCamera(
            prim_path=self.camera_path,
            position=np.array([0.8916, -1.4017, 3.139]),
            frequency=30,
            resolution=(640, 480),
            orientation=target_orientation,
        )

    def setup_scene(self):
        self.world.scene.add_default_ground_plane()

    def initialize(self):
        if self._initialized:
            return
        self.camera.initialize()
        self._initialized = True

    def step(self):
        self.world.step(render=True)
        if self.world.is_playing():
            rgba = self.camera.get_rgba()
            return rgba
        return None


class Lidar:
    def __init__(
        self,
        robot_path,
        lidar_name="LidarName",
        min_range=0.4,
        max_range=20.0,
        draw_points=True,
        horizontal_fov=360.0,
        vertical_fov=60.0,
        enable_semantics=True,
        rotation_rate=0,
        horizontal_resolution=1.0,
        vertical_resolution=1.0,
        translation=Gf.Vec3d(0.0, 0.0, 0.1),
    ):
        self.robot_path = robot_path
        self.lidar_name = lidar_name
        self.min_range = min_range
        self.max_range = max_range
        self.draw_points = draw_points
        self.horizontal_fov = horizontal_fov
        self.vertical_fov = vertical_fov
        self.enable_semantics = enable_semantics
        self.rotation_rate = rotation_rate
        self.horizontal_resolution = horizontal_resolution
        self.vertical_resolution = vertical_resolution
        self.translation = translation
        self.full_path = f"{self.robot_path}/{self.lidar_name}"

    def setup(self):
        import omni.kit.commands
        import omni.usd

        omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path=f"/{self.lidar_name}",
            parent=self.robot_path,
            min_range=self.min_range,
            max_range=self.max_range,
            draw_points=self.draw_points,
            horizontal_fov=self.horizontal_fov,
            vertical_fov=self.vertical_fov,
            enable_semantics=self.enable_semantics,
            rotation_rate=self.rotation_rate,
            horizontal_resolution=self.horizontal_resolution,
            vertical_resolution=self.vertical_resolution,
        )
        if self.translation is not None:
            stage = omni.usd.get_context().get_stage()
            lidar_prim = stage.GetPrimAtPath(self.full_path)
            if lidar_prim.IsValid():
                translate_attr = lidar_prim.GetAttribute("xformOp:translate")
                if not translate_attr:
                    UsdGeom.Xformable(lidar_prim).AddTranslateOp()
                    translate_attr = lidar_prim.GetAttribute("xformOp:translate")
                translate_attr.Set(self.translation)
        return self.full_path


def setup_human_semantics(stage, human_path="/World/male", semantic_label="human"):
    human_prim = stage.GetPrimAtPath(human_path)
    if not human_prim.IsValid():
        return False
    for prim in Usd.PrimRange(human_prim):
        if prim.IsA(UsdGeom.Mesh):
            add_update_semantics(prim=prim, semantic_label=semantic_label)
    return True
