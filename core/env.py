import os

from omni.isaac.kit import SimulationApp
from pxr import Gf, Usd, Sdf, UsdPhysics, PhysxSchema, UsdGeom
import numpy as np
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import SurfaceGripper
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.rotations import euler_angles_to_quat

from isaacsim.core.prims import GeometryPrim, RigidPrim
from core.bolt_nut import Bolt, Nut

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
    
    # 볼트 에셋 불러오는 함수
    def add_bolt(self, bolt_usd_path, position=np.array([0.7931, 2.0, 0.88053])):
        # if not os.path.exists(bolt_usd_path):
        #     base_path, _ = os.path.splitext(bolt_usd_path)
        #     for ext in (".usdc", ".usda", ".usd"):
        #         alt_path = f"{base_path}{ext}"
        #         if os.path.exists(alt_path):
        #             print(f"Warning: {bolt_usd_path} not found, using {alt_path}")
        #             bolt_usd_path = alt_path
        #             break
        #     else:
        #         raise FileNotFoundError(f"USD file not found at {bolt_usd_path}")

        add_reference_to_stage(usd_path=bolt_usd_path, prim_path="/World/Bolt")
        
        
        # bolt_prim = self.stage.GetPrimAtPath("/World/Bolt")
        # if not bolt_prim.IsValid():
        #     bolt_prim = self.stage.DefinePrim("/World/Bolt", "Xform")
        #     bolt_prim.GetReferences().AddReference(bolt_usd_path)
        
        # if bolt_prim.IsValid():
        #     # 물리 속성 적용 (bolt_cad.usd용)
        #     if not bolt_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        #         UsdPhysics.RigidBodyAPI.Apply(bolt_prim)
        #     for prim in Usd.PrimRange(bolt_prim):
        #         if prim.GetTypeName() == "Mesh":
        #             UsdPhysics.CollisionAPI.Apply(prim)
        #             physx_api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        #             if hasattr(physx_api, "CreateApproximationAttr"):
        #                 physx_api.CreateApproximationAttr().Set("convexHull")
        #             elif hasattr(physx_api, "GetApproximationAttr"):
        #                 attr = physx_api.GetApproximationAttr()
        #                 if not attr:
        #                     attr = prim.CreateAttribute(
        #                         "physxCollision:approximation", Sdf.ValueTypeNames.Token
        #                     )
        #                 attr.Set("convexHull")
        #             else:
        #                 prim.CreateAttribute(
        #                     "physxCollision:approximation", Sdf.ValueTypeNames.Token
        #                 ).Set("convexHull")

        #     self.world.scene.add(
        #         RigidPrim(
        #             prim_paths_expr="/World/Bolt",
        #             name="my_bolt",
        #             positions=np.array([position]),
        #             scales=np.array([[2, 2, 2]]),
        #             orientations=np.array([euler_angles_to_quat(np.array([-np.pi/2, 0, 0]))]),
        #         )
        #     )

        self.bolt = self.world.scene.add(
                RigidPrim(
                    prim_paths_expr="/World/Bolt",
                    name="my_bolt",
                    positions=np.array([position]),
                    scales=np.array([[2, 2, 2]]),
                    orientations=np.array([euler_angles_to_quat(np.array([-np.pi/2, 0, 0]))]),
                ))
        
        return self.bolt
    
    # 너트 에셋 불러오는 함수
    def add_nut(self, nut_usd_path, position=np.array([[0.5, 2.0, 0.9]])):
        add_reference_to_stage(usd_path=nut_usd_path, prim_path="/World/Nut")
        

        self.nut = self.world.scene.add(
            RigidPrim(
                prim_paths_expr="/World/Nut",
                name = "my_nut",
                positions = position,
                scales = np.array([[2, 2, 2]])
            )
        )

        return self.nut

    # 볼트, 너트 자체 제작 함수
    def create_bolts_and_nuts(
        self,
        bolt_count=3,
        nut_count=1,
        position=Gf.Vec3d(0.834, 3.0, 1.0),
        spawn_interval=10000.0,     # 20초마다 생성
    ):


        import omni.kit.app
        import omni.timeline

        self._spawn_base_position = position
        self._spawn_interval = spawn_interval
        self._spawn_index = 0
        self._spawn_next_is_bolt = bolt_count >= nut_count

        timeline = omni.timeline.get_timeline_interface()

        def _spawn_once():
            current_position = self._spawn_base_position

            if self._spawn_next_is_bolt:
                Bolt().create(self.stage, current_position)
            else:
                Nut().create(self.stage, current_position)

            self._spawn_next_is_bolt = not self._spawn_next_is_bolt
            self._spawn_index += 1

        _spawn_once()
        self._spawn_last_time = timeline.get_current_time()

        def _on_update(_event):
            if not timeline.is_playing():
                return
            current_time = timeline.get_current_time()
            if current_time < self._spawn_last_time:
                self._spawn_last_time = current_time
                return
            if current_time - self._spawn_last_time < self._spawn_interval:
                return
            _spawn_once()
            self._spawn_last_time = current_time

        app = omni.kit.app.get_app()
        self._spawn_subscription = app.get_update_event_stream().create_subscription_to_pop(
            _on_update
        )

    def get_box_position(self, name, prim_path):
        box_prim = self.stage.GetPrimAtPath(prim_path)
        box = self.world.scene.get_object(name) 
        if box is not None:
            pose, _ = box.get_world_poses()
            return pose[0] if len(pose) > 0 else None
        
        xform_cache = UsdGeom.XformCache()
        transform = xform_cache.GetLocalToWorldTransform(box_prim)
        translation = transform.ExtractTranslation()
        # 박스 0.5 높은 위치 반환
        return np.array([translation[0], translation[1], translation[2]+0.5], dtype=np.float64)
