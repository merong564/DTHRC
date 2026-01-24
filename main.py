# SimulationApp must be created first
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni
from isaacsim.sensors.physx import _range_sensor
from isaacsim.util.debug_draw import _debug_draw
import numpy as np

from core.env import EnvManager
from core.safety import SafetyManager
from core.move import HumanController
from core.pick_and_place import RobotController
from core.perception import select_bolt_prim_path, draw_detection_bboxes
from core.sensor import Camera, Lidar, setup_human_semantics
from core.bolt_nut import create_bolts_and_nut
from core.yolo import YoloDetector

BOLT_PRIM_PATH = None


def main():
    usd_path = "/home/rokey/Desktop/DTHRC/DTHRC/assets/env_gripper.usd"
    robot_usd_path = "/home/rokey/Desktop/DTHRC/DTHRC/assets/ur10/ur10.usd"
    bolt_usd_path = "/home/rokey/Desktop/DTHRC/DTHRC/assets/bolt_cad2.usd"

    env = EnvManager(usd_path, robot_usd_path)
    stage = env.stage
    my_world = env.world
    my_robot = env.add_robot(robot_usd_path)
    my_world.reset()
    env.add_bolt(bolt_usd_path)

    create_bolts_and_nut(stage, BOLT_PRIM_PATH, bolt_count=3)

    timeline = omni.timeline.get_timeline_interface()
    lidar_interface = _range_sensor.acquire_lidar_sensor_interface()
    placing_position = np.array([-1.25, -0.25047, 1.5])

    lidar = Lidar(robot_path="/World/UR10/base_link")
    lidar.setup()
    setup_human_semantics(stage, human_path="/World/male")

    safety = SafetyManager(stage, lidar_interface)
    human_control = HumanController(stage, "/World/male")
    robot_controller = RobotController(my_world, my_robot, placing_position)

    camera_sensor = Camera()
    camera_sensor.setup_scene()

    yolo_engine = YoloDetector()
    debug_draw = _debug_draw.acquire_debug_draw_interface()
    depth_m = 2.0
    detected_bolt_prim_path = None

    timeline.play()
    frame_count = 0

    for _ in range(10):
        simulation_app.update()

    camera_sensor.initialize()

    try:
        while simulation_app.is_running():
            simulation_app.update()

            rgba_data = camera_sensor.step()
            if rgba_data is not None and rgba_data.size > 0:
                results = yolo_engine.detect(rgba_data)
                if results:
                    new_bolt_prim_path = select_bolt_prim_path(
                        stage, camera_sensor.camera, results, depth_m
                    )
                    if (
                        new_bolt_prim_path
                        and new_bolt_prim_path != detected_bolt_prim_path
                    ):
                        detected_bolt_prim_path = new_bolt_prim_path
                        robot_controller.set_bolt_prim_path(
                            detected_bolt_prim_path
                        )
                    draw_detection_bboxes(
                        debug_draw, camera_sensor.camera, results, depth_m
                    )
                else:
                    print("#### No detections. ####")

            if frame_count > 60:
                current_time = timeline.get_current_time()

                robot_controller.control_robot()

                dist = safety.get_human_distance()
                safety.update_led_for_distance(dist)

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
