# 반드시 SimulationApp이 가장 먼저 생성되어야 함
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni
from isaacsim.sensors.physx import _range_sensor
from core.env import EnvManager
from core.safety import SafetyManager
from core.move import HumanController
from core.pick_n_place import RobotController

def main():
    usd_path = "/home/rokey/Desktop/DTHRC/DTHRC/assets/env_gripper.usd"
    robot_usd_path = "/home/rokey/Desktop/DTHRC/DTHRC/assets/ur10/ur10.usd"
    
    # 1. 환경 관리자 초기화
    env = EnvManager(usd_path, robot_usd_path)
    env.setup_physics()
    
    stage = env.stage
    my_world = env.world # EnvManager의 world 가져오기
    my_robot = env.robot # EnvManager에서 생성된 로봇 가져오기

    timeline = omni.timeline.get_timeline_interface()
    lidar_interface = _range_sensor.acquire_lidar_sensor_interface()

    # 2. 각 모듈 초기화
    safety = SafetyManager(stage, lidar_interface)
    human_control = HumanController(stage, "/World/male")
    print("제어기 가져오기 전")
    robot_controller = RobotController(my_world, my_robot)
    print("제어기 가져오기 후")

    # 3. 루프 변수
    timeline.play()
    frame_count = 0
  

    try:
        while simulation_app.is_running():
            simulation_app.update()
            
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
                robot_controller.control_robot()

            frame_count += 1

    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        timeline.stop()
        simulation_app.close()

if __name__ == "__main__":
    main()