# 반드시 SimulationApp이 가장 먼저 생성되어야 함
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.util.debug_draw import _debug_draw

import omni
from isaacsim.sensors.physx import _range_sensor
from core.env import EnvManager
from core.safety import SafetyManager
from core.move import HumanController
from core.perception import Camera
from core.yolo import YoloDetector


def _bbox_corners_xyxy(bbox):
    x1, y1, x2, y2 = bbox
    return np.array(
        [
            [x1, y1],
            [x2, y1],
            [x2, y2],
            [x1, y2],
        ],
        dtype=np.float32,
    )


def _bbox_lines_world(camera_sensor, bbox, depth_m):
    points_2d = _bbox_corners_xyxy(bbox)
    depth = np.full((4,), depth_m, dtype=np.float32)
    points_3d = camera_sensor.get_world_points_from_image_coords(points_2d, depth)
    edges = [(0, 1), (1, 2), (2, 3), (3, 0)]
    starts = [tuple(points_3d[i]) for i, _ in edges]
    ends = [tuple(points_3d[j]) for _, j in edges]
    return starts, ends


def main():
    usd_path = "/home/rokey/Desktop/DTHRC/DTHRC/assets/env_gripper.usd"
    
    
    # 1. 환경 관리자 초기화
    env = EnvManager(usd_path)
    env.setup_physics()
    
    stage = env.stage
    timeline = omni.timeline.get_timeline_interface()
    lidar_interface = _range_sensor.acquire_lidar_sensor_interface()

    # 2. 각 모듈 초기화
    safety = SafetyManager(stage, lidar_interface)
    human_control = HumanController(stage, "/World/male")

    
    camera_test = Camera()
    camera_test.setup_scene()

    yolo_engine = YoloDetector()
    debug_draw = _debug_draw.acquire_debug_draw_interface()
    depth_m = 2.0

    # 3. 루프 변수
    timeline.play()
    frame_count = 0

    for _ in range(10):
        simulation_app.update()

    camera_test.initialize()

  

    try:
        while simulation_app.is_running():
            simulation_app.update()

            rgba_data = camera_test.step()
            if rgba_data is not None and rgba_data.size > 0:
                results = yolo_engine.detect(rgba_data)
                debug_draw.clear_lines()
                if results:
                    line_starts = []
                    line_ends = []
                    colors = []
                    sizes = []
                    for res in results:
                        starts, ends = _bbox_lines_world(camera_test.camera, res["bbox"], depth_m)
                        line_starts.extend(starts)
                        line_ends.extend(ends)
                        color = (0.0, 1.0, 0.0, 1.0) if res["class"] == "bolt" else (1.0, 0.6, 0.0, 1.0)
                        colors.extend([color] * 4)
                        sizes.extend([2.0] * 4)
                    debug_draw.draw_lines(line_starts, line_ends, colors, sizes)
            
            if frame_count > 60:
                current_time = timeline.get_current_time()
                
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

            frame_count += 1

    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        timeline.stop()
        _debug_draw.release_debug_draw_interface(debug_draw)
        simulation_app.close()

if __name__ == "__main__":
    main()
