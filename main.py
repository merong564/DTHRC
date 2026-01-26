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
from core.perception import select_target_prim_path, draw_detection_bboxes
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
    # bolt_usd_path = "/home/rokey/Desktop/DTHRC/DTHRC/assets/factory_bolt_m20_loose/factory_bolt_m20_loose.usd"
    # nut_usd_path = "/home/rokey/Desktop/DTHRC/DTHRC/assets/factory_nut_m20_loose/factory_nut_m20_loose_2.usd"

    # 1. 환경 관리자 초기화
    env = EnvManager(usd_path, robot_usd_path)
    # env.setup_physics()
    
    stage = env.stage
    my_world = env.world # EnvManager의 world 가져오기
    my_robot = env.add_robot(robot_usd_path) # EnvManager에서 생성된 로봇 가져오기
    # my_bolt = env.add_bolt(bolt_usd_path)
    # my_nut = env.add_nut(nut_usd_path)
    my_world.reset()
    
    # pick_and_place_gripper.py 실행 시
    bolt_count = 1
    nut_count = 1
    position = Gf.Vec3d(0.834, 3.0, 1.0)
    spawn_interval = 200.0
    # offset = 0.0
    env.create_bolts_and_nuts(
        BOLT_PRIM_PATH,
        bolt_count=bolt_count,
        nut_count=nut_count,
        position=position,
        spawn_interval=spawn_interval
        # offset=offset,
    )

    timeline = omni.timeline.get_timeline_interface()
    lidar_interface = _range_sensor.acquire_lidar_sensor_interface()

    # 너트 집을 시 수정 필요
    bolt_placing_position = env.get_box_position('bolt_box', '/World/bolt_box')
    nut_placing_position = env.get_box_position('nut_box', '/World/nut_box')
    placing_position = bolt_placing_position

    # 라이다 설정
    lidar = Lidar(robot_path="/World/UR10/base_link")
    lidar.setup()
    setup_human_semantics(stage, human_path="/World/male")

    safety = SafetyManager(stage, lidar_interface)
    human_control = HumanController(stage, "/World/male")

    # pick_and_place_follow.py 실행 시
    # robot_controller = RobotController(my_world, my_robot, placing_position, my_bolt)

    # pick_and_place_gripper.py 실행 시
    robot_controller = RobotController(my_world, my_robot, placing_position)

    camera_sensor = Camera()
    camera_sensor.setup_scene()
    camera_sensor.initialize()

    yolo_engine = YoloDetector()
    debug_draw = _debug_draw.acquire_debug_draw_interface()
    depth_m = 2.0

    desired_class = "bolt"   # "bolt" -> "nut" 순서 강제
    detected_target_prim_path = None

    timeline.play()
    frame_count = 0

    # current_target_obj = my_bolt
    # current_target_type = "bolt"  # pick_and_place_follow.py 실행 시

    # for _ in range(10):
    #     simulation_app.update()

    try:
        while simulation_app.is_running():
            simulation_app.update()

            detected_target_obj = None
            rgba_data = camera_sensor.step()
            if rgba_data is not None and rgba_data.size > 0:
                results = yolo_engine.detect(rgba_data)
                if results:
                    # ✅ "원하는 클래스"만 고른다 (bolt 먼저, 그 다음 nut)
                    target_prim_path = select_target_prim_path(
                        stage,
                        camera_sensor.camera,
                        results,
                        depth_m,
                        target_class=desired_class,          # 핵심!
                        min_confidence=0.25,
                        center_ratio=1.0,
                    )

                    # pick&place가 끝났거나(phase>=6) 아직 타겟이 없을 때만 타겟 갱신
                    if target_prim_path and (detected_target_prim_path is None or robot_controller.task_phase >= 6):
                        detected_target_prim_path = target_prim_path
                        robot_controller.set_bolt_prim_path(detected_target_prim_path)  # 이름은 bolt지만 nut도 OK (prim_path 기반)

                        # ✅ placing 위치는 현재 desired_class로 결정
                        if desired_class == "bolt":
                            placing_position = bolt_placing_position
                        else:
                            placing_position = nut_placing_position

                        robot_controller._placing_position = placing_position

                        # 만약 이전 사이클이 끝난 상태면 새 사이클 시작
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
                robot_controller.control_robot(speed_ratio=speed_ratio)
                
                if robot_controller.task_phase == 6:
                    # ✅ bolt -> nut 로 전환 (원하면 다시 bolt로 돌아가게도 가능)
                    if desired_class == "bolt":
                        desired_class = "nut"
                        print("[sequence] next target: NUT")
                    else:
                        desired_class = "bolt"
                        print("[sequence] next target: BOLT")

                    detected_target_prim_path = None     # pick and place
                

                # pick_and_place_follow.py 실행 시
                # if robot_controller.task_phase == 7:
                #     if current_target_type == "bolt":
                #         robot_controller.set_target(my_nut) # 컨트롤러 내부 타겟을 Nut으로 변경
                #         current_target_type = "nut"
                #     elif current_target_type == "nut":
                #         pass

            frame_count += 1

    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        timeline.stop()
        _debug_draw.release_debug_draw_interface(debug_draw)
        simulation_app.close()

if __name__ == "__main__":
    main()



