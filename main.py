# 반드시 SimulationApp이 가장 먼저 생성되어야 함
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni
from isaacsim.sensors.physx import _range_sensor
from core.env import EnvManager
from core.safety import SafetyManager
from core.move import HumanController
from core.pick_and_place import RobotController
import numpy as np

# yolo 관련 import
from isaacsim.util.debug_draw import _debug_draw
from pxr import Gf, UsdShade, Sdf
from core.bolt_nut import Bolt, Nut
from core.utils import bbox_lines_world, find_bolt_prim, get_world_translation
from core.env import EnvManager
from core.safety import SafetyManager
from core.move import HumanController
from core.perception import Camera
from core.yolo import YoloDetector
BOLT_PRIM_PATH = None

def _get_or_create_material(stage, material_path, color):
    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, f"{material_path}/Shader")
    shader.CreateIdAttr("OmniPBR")
    shader.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(color)
    shader.CreateInput("roughness_constant", Sdf.ValueTypeNames.Float).Set(0.4)
    shader.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(0.0)
    material.CreateSurfaceOutput().ConnectToSource(shader, "surface")
    return material


def _bind_material_to_prim(stage, prim_path, material):
    prim = stage.GetPrimAtPath(prim_path)
    if prim.IsValid():
        UsdShade.MaterialBindingAPI(prim).Bind(material)

# MOD: map YOLO bolt detection to closest bolt prim in stage
def _select_bolt_prim_path(stage, camera_sensor, detections, depth_m):
    bolt_dets = [d for d in detections if d.get("class") == "bolt"]
    if not bolt_dets:
        return None
    best_det = max(bolt_dets, key=lambda d: d.get("confidence", 0.0))
    center = np.array([best_det["center"]], dtype=np.float32)
    depth = np.array([depth_m], dtype=np.float32)
    world_point = camera_sensor.get_world_points_from_image_coords(center, depth)[0]
    candidates = []
    for prim in stage.Traverse():
        name = prim.GetName().lower()
        path = prim.GetPath().pathString.lower()
        if "bolt" in name or "bolt" in path:
            candidates.append(prim)
    if not candidates:
        return None
    best_prim = min(
        candidates,
        key=lambda p: np.linalg.norm(np.array(get_world_translation(p)) - world_point),
    )
    return best_prim.GetPath().pathString

def main():
    usd_path = "/home/rokey/Desktop/DTHRC/DTHRC/assets/env_gripper.usd"
    robot_usd_path = "/home/rokey/Desktop/DTHRC/DTHRC/assets/ur10/ur10.usd"
    bolt_usd_path = "/home/rokey/Desktop/DTHRC/DTHRC/assets/bolt_cad2.usd"
    
    
    # 1. 환경 관리자 초기화
    env = EnvManager(usd_path, robot_usd_path)
    # env.setup_physics()
    
    stage = env.stage
    my_world = env.world # EnvManager의 world 가져오기
    my_robot = env.add_robot(robot_usd_path) # EnvManager에서 생성된 로봇 가져오기
    my_world.reset()
    env.add_bolt(bolt_usd_path)
    print("1111111111111111111111111")

    # yolo 볼트 추가
    bolt_builders = [Bolt() for _ in range(3)] # 3개의 볼트 생성
    nut_builder = Nut()
    bolt_prim = stage.GetPrimAtPath(BOLT_PRIM_PATH) if BOLT_PRIM_PATH else find_bolt_prim(stage)
    if bolt_prim and bolt_prim.IsValid():
        bolt_pos = get_world_translation(bolt_prim)
        hex_center = Gf.Vec3d(bolt_pos[0], bolt_pos[1], bolt_pos[2] + nut_builder.z_offset)
    else:
        print("Bolt prim not found; placing hexagon at world origin with z offset.")
        hex_center = Gf.Vec3d(0.0, 0.0, nut_builder.z_offset)
    nut_builder.create(stage, hex_center)
    for i, bolt_builder in enumerate(bolt_builders):
        bolt_base_pos = hex_center + bolt_builder.offset_from_nut + Gf.Vec3d(-0.4 * i, 0.0, 0.0) # 생성되는 볼트 간격 조정
        bolt_builder.create(stage, bolt_base_pos)


    timeline = omni.timeline.get_timeline_interface()
    lidar_interface = _range_sensor.acquire_lidar_sensor_interface()
    placing_position = np.array([-1.25, -0.25047, 1.5])
    # 2. 각 모듈 초기화
    safety = SafetyManager(stage, lidar_interface)
    human_control = HumanController(stage, "/World/male")
    print("제어기 가져오기 전")
    robot_controller = RobotController(my_world, my_robot,placing_position)
    print("제어기 가져오기 후")

    # yolo 카메라
    camera_test = Camera()
    camera_test.setup_scene()

    yolo_engine = YoloDetector()
    debug_draw = _debug_draw.acquire_debug_draw_interface()
    depth_m = 2.0
    detected_bolt_prim_path = None  # MOD: track YOLO-selected bolt prim

    # 3. 루프 변수
    timeline.play()
    frame_count = 0

    # 카메라 업데이트
    for _ in range(10):
        simulation_app.update()

    camera_test.initialize()

    try:
        while simulation_app.is_running():
            simulation_app.update()

            # 카메라
            rgba_data = camera_test.step()
            if rgba_data is not None and rgba_data.size > 0:
                results = yolo_engine.detect(rgba_data)
                debug_draw.clear_lines()
                if results:
                    new_bolt_prim_path = _select_bolt_prim_path(  # MOD
                        stage, camera_test.camera, results, depth_m
                    )
                    # print("############ Detections: ############")
                    if (
                        new_bolt_prim_path
                        and new_bolt_prim_path != detected_bolt_prim_path
                    ):
                        detected_bolt_prim_path = new_bolt_prim_path  # MOD
                        robot_controller.set_bolt_prim_path(  # MOD
                            detected_bolt_prim_path
                        )
                    line_starts = []
                    line_ends = []
                    colors = []
                    sizes = []
                    for res in results:
                        starts, ends = bbox_lines_world(camera_test.camera, res["bbox"], depth_m)
                        line_starts.extend(starts)
                        line_ends.extend(ends)
                        color = (0.0, 1.0, 0.0, 1.0) if res["class"] == "bolt" else (1.0, 0.6, 0.0, 1.0)
                        colors.extend([color] * 4)
                        sizes.extend([2.0] * 4)
                    debug_draw.draw_lines(line_starts, line_ends, colors, sizes)

                else:
                    print("#### No detections. ####")
            
            if frame_count > 60:
                current_time = timeline.get_current_time()

                robot_controller.control_robot()
                
                # 거리 측정 및 로직 판단
                dist = safety.get_human_distance()
                
                if dist is not None:
                    if dist < 1.4:
                        safety.change_led_color(1.0, 0.0, 0.0) # Red
                        
                    elif dist < 2.2:
                        safety.change_led_color(1.0, 1.0, 0.0) # Yellow
                        
                    else:
                        safety.change_led_color(0.0, 0.0, 1.0) # Blue
                        
                else:
                    safety.change_led_color(0.0, 0.0, 1.0) # Blue (No detection)
                    

                # 사람 이동 업데이트
                human_control.move_human(current_time)
                robot_controller.control_robot()

            frame_count += 1

    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        timeline.stop()
        _debug_draw.release_debug_draw_interface(debug_draw)
        simulation_app.close()

if __name__ == "__main__":
    main()
