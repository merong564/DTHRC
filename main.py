# 반드시 SimulationApp이 가장 먼저 생성되어야 함
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni
from isaacsim.sensors.physx import _range_sensor
from core.env import EnvManager
from core.safety import SafetyManager
from core.move import HumanController
from core.pick_and_place import RobotController
# from core.pnp import RobotController
import numpy as np

def main():
    usd_path = "/home/rokey/Desktop/DTHRC/DTHRC/assets/env_gripper.usd"
    robot_usd_path = "/home/rokey/Desktop/DTHRC/DTHRC/assets/ur10/ur10.usd"
    bolt_usd_path = "/home/rokey/Desktop/DTHRC/DTHRC/assets/factory_bolt_m20_loose/factory_bolt_m20_loose.usd"
    nut_usd_path = "/home/rokey/Desktop/DTHRC/DTHRC/assets/factory_nut_m20_loose/factory_nut_m20_loose_2.usd"
    
    
    # 1. 환경 관리자 초기화
    env = EnvManager(usd_path, robot_usd_path)
    # env.setup_physics()
    
    stage = env.stage
    my_world = env.world # EnvManager의 world 가져오기
    my_robot = env.add_robot(robot_usd_path) # EnvManager에서 생성된 로봇 가져오기
    my_bolt = env.add_bolt(bolt_usd_path)
    my_nut = env.add_nut(nut_usd_path)
    my_world.reset()

    timeline = omni.timeline.get_timeline_interface()
    lidar_interface = _range_sensor.acquire_lidar_sensor_interface()
    placing_position = np.array([-0.90365, -0.25047, 1.3])
    # 2. 각 모듈 초기화
    safety = SafetyManager(stage, lidar_interface)
    human_control = HumanController(stage, "/World/male")

    robot_controller = RobotController(my_world, my_robot,placing_position,my_bolt)


    # 3. 루프 변수
    timeline.play()
    frame_count = 0
    current_target_type = "bolt"

    try:
        while simulation_app.is_running():
            simulation_app.update()
            
            if frame_count > 60:
                current_time = timeline.get_current_time()

                human_control.move_human(current_time)

                speed_ratio = 1.0
                
                # 거리 측정 및 로직 판단
                dist = safety.get_human_distance()
                
                if dist is not None:
                    if dist < 1.4:
                        safety.change_led_color(1.0, 0.0, 0.0) # Red
                        speed_ratio = 0.0
                        
                    elif dist < 2.2:
                        safety.change_led_color(1.0, 1.0, 0.0) # Yellow
                        speed_ratio = 0.15
                    else:
                        safety.change_led_color(0.0, 0.0, 1.0) # Blue
                        speed_ratio = 1.0
                else:
                    safety.change_led_color(0.0, 0.0, 1.0) # sky (No detection)
                    speed_ratio = 1.0
                    
                robot_controller.control_robot(speed_ratio=speed_ratio)
                if robot_controller.task_phase == 9:
                    if current_target_type == "bolt":
                        robot_controller.set_target(my_nut) # 컨트롤러 내부 타겟을 Nut으로 변경
                        current_target_type = "nut"
                    elif current_target_type == "nut":
                        pass

            frame_count += 1

    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        timeline.stop()
        simulation_app.close()

if __name__ == "__main__":
    main()