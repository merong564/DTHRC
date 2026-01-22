
import sys
import os
import numpy as np
 

# root_path를 utils 폴더 내부까지 직접 지정
controller_path = "/home/rokey/Desktop/DTHRC/DTHRC/utils/controller"
tasks_path = "/home/rokey/Desktop/DTHRC/DTHRC/utils/tasks"

sys.path.insert(0, controller_path)
sys.path.insert(0, tasks_path)

# 이제 'utils.' 없이 바로 파일 이름으로 호출

import omni
from omni.isaac.core import World
from rmpflow import RMPFlowController
from follow_target import FollowTargetCustom
from omni.isaac.core.utils.xforms import get_world_pose


# --- 설정 및 경로 ---
usd_path = "/home/rokey/Desktop/DTHRC/DTHRC/assets/env_gripper.usd"
robot_path = "/World/ur10e"
lidar_full_path = f"{robot_path}/LidarName"
human_path = "/World/male"
danger_path = "/World/danger"


class RobotController:
    def __init__(self, world, robot):
        self.my_world = world
        self.robot = robot

        # 1. 로봇 태스크 및 컨트롤러 설정
        self.my_task = FollowTargetCustom(
            name="bolt_task",
            target_prim_path = "/World/bolt",
            target_name = "bolt_target",
            target_position=np.array([1.0, 0, 0.9])
            # robot_prim_path=self.robot.prim_path,
            # robot_name = "/World/ur10e"
            )
        self.my_world.add_task(self.my_task)
        self.my_world.reset()

        

        # bolt_position, bolt_orientation = get_world_pose(prim_path="/World/bolt")
        # self.my_task.set_params(
        #     "/World/bolt",
        #     "target_bolt", 
        #     bolt_position, 
        #     bolt_orientation
        # )
        print('get_params 전')
        task_params = self.my_task.get_params()
        print("task_params: ", task_params)
        self.target_name = task_params["target_name"]["value"]
        
        # try:
        #     task_params = self.my_task.get_params()
        #     self.target_name = task_params["target_name"]["value"]
        # except Exception as e:
        #     print(f"매개변수 로드 실패: {e}")
        #     self.target_name = "target_bolt"
        
        # task_params = self.my_task.get_params()
        # # # task_params = self.my_world.get_task("/World/bolt").get_params()
        # self.target_name = task_params["target_name"]["value"]

        self.my_controller = RMPFlowController(
            name="target_follower_controller", 
            robot_articulation=self.robot,
            attach_gripper = True
            )
        self.articulation_controller = self.robot.get_articulation_controller()

    def control_robot(self):
        observations = self.my_world.get_observations()
        if self.target_name in observations:
            actions = self.my_controller.forward(
                target_end_effector_position=observations[self.target_name]["position"],
                target_end_effector_orientation=observations[self.target_name]["orientation"],
            )
            self.articulation_controller.apply_action(actions)
    
            