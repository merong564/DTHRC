# 반드시 SimulationApp이 가장 먼저 생성되어야 함
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni
from isaacsim.sensors.physx import _range_sensor
from core.env import EnvManager
from core.safety import SafetyManager
from core.move import HumanController
# from core.pick_and_place_follow import RobotController              # pick_and_place_follow.py 실행 시
from core.pick_and_place_gripper import RobotController           # # pick_and_place_gripper.py 실행 시
from core.perception import select_best_target_prim_path, draw_detection_bboxes
from core.sensor import Camera, Lidar, setup_human_semantics
from core.yolo import YoloDetector
from pxr import Gf
from isaacsim.util.debug_draw import _debug_draw
import numpy as np

BOLT_PRIM_PATH = None


def _select_best_detection(detections):
    return max(detections, key=lambda d: d.get("confidence", 0.0))


def main():
    usd_path = "/home/rokey/Desktop/DTHRC/DTHRC/assets/env_final.usd"
    robot_usd_path = "/home/rokey/Desktop/DTHRC/DTHRC/assets/ur10/ur10.usd"

    # 1. 환경 관리자 초기화
    env = EnvManager(usd_path, robot_usd_path)
    
    stage = env.stage
    my_world = env.world # EnvManager의 world 가져오기
    my_robot = env.add_robot(robot_usd_path) # EnvManager에서 생성된 로봇 가져오기
    my_world.reset()
    
    bolt_count = 1
    nut_count = 1
    position = Gf.Vec3d(0.834, 2.0, 1.0)
    spawn_interval = 300.0
    env.create_bolts_and_nuts(
        bolt_count=bolt_count,
        nut_count=nut_count,
        position=position,
        spawn_interval=spawn_interval
    )

    timeline = omni.timeline.get_timeline_interface()
    lidar_interface = _range_sensor.acquire_lidar_sensor_interface()

    bolt_placing_position = env.get_box_position('bolt_box', '/World/bolt_box')
    nut_placing_position = env.get_box_position('nut_box', '/World/nut_box')
    placing_position = bolt_placing_position

    # 라이다 설정
    lidar = Lidar(robot_path="/World/UR10/base_link")
    lidar.setup()
    setup_human_semantics(stage, human_path="/World/male")

    safety = SafetyManager(stage, lidar_interface)
    human_control = HumanController(stage, "/World/male")

    robot_controller = RobotController(my_world, my_robot, placing_position)

    camera_sensor = Camera()
    camera_sensor.setup_scene()
    camera_sensor.initialize()

    yolo_engine = YoloDetector()
    debug_draw = _debug_draw.acquire_debug_draw_interface()
    depth_m = 2.0

    detected_target_prim_path = None

    timeline.play()
    frame_count = 0


    try:
        while simulation_app.is_running():
            simulation_app.update()

            results = []
            rgba_data = camera_sensor.step()
            if rgba_data is not None and rgba_data.size > 0:
                results = yolo_engine.detect(rgba_data)
                if results:
                    best_target_prim_path, best_target_class = select_best_target_prim_path(
                        stage,
                        camera_sensor.camera,
                        results,
                        depth_m,
                        base_prim_path="/World/UR10/base_link",
                    )
                    if best_target_prim_path and (
                        detected_target_prim_path is None
                        or robot_controller.task_phase >= 6
                    ):
                        detected_target_prim_path = best_target_prim_path
                        robot_controller.set_bolt_prim_path(
                            detected_target_prim_path
                        )
                        if best_target_class == "bolt":
                            placing_position = bolt_placing_position
                        elif best_target_class == "nut":
                            placing_position = nut_placing_position
                        robot_controller._placing_position = placing_position
                        if robot_controller.task_phase >= 6:
                            robot_controller.task_phase = 1
                            robot_controller.my_controller.reset()
                        
                    draw_detection_bboxes(
                        debug_draw, camera_sensor.camera, results, depth_m
                    )
                else:
                    print("#### No detections. ####")

            if frame_count > 60:
                current_time = timeline.get_current_time()

                human_control.move_human(current_time)
                
                dist = safety.get_human_distance()                          # 거리 측정
                print(f'human distance: {dist}')
                speed_ratio = safety.update_led_for_distance(dist)          # 거리에 따른 로봇 속도(감속/정지) 설정
                print(f'speed_ratio: {speed_ratio}')
                if robot_controller.task_phase < 6:
                    robot_controller.control_robot(speed_ratio=speed_ratio)
                

            frame_count += 1

    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        timeline.stop()
        _debug_draw.release_debug_draw_interface(debug_draw)
        simulation_app.close()

if __name__ == "__main__":
    main()
