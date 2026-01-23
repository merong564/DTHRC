from omni.isaac.kit import SimulationApp
import os
import math
import numpy as np

# 1. 시뮬레이션 앱 초기화
simulation_app = SimulationApp({"headless": False}) 

import omni
from isaacsim.sensors.physx import _range_sensor
from pxr import UsdGeom, Gf, Usd, UsdShade
from omni.isaac.core.utils.semantics import add_update_semantics
from omni.isaac.core import World

from controller.rmpflow import RMPFlowController # RMPFlow 컨트롤러
from tasks.follow_target import FollowTarget # 대상 추적 태스크
from isaacsim.robot.manipulators.grippers import SurfaceGripper

# --- 설정 및 경로 ---
usd_path = "/home/rokey/Desktop/DTHRC/env_default.usd"
robot_path = "/World/ur10e"
lidar_full_path = f"{robot_path}/LidarName"
human_path = "/World/male"
danger_path = "/World/danger"

# --- 전역 변수 및 상태 제어 ---
is_robot_stopped = False

# --- 함수 정의 ---

def change_color(r, g, b, intensity=10000.0):
    """danger 객체의 Emissive 색상을 변경하는 함수"""
    danger_prim = stage.GetPrimAtPath(danger_path)
    if danger_prim.IsValid():
        mat_binding = UsdShade.MaterialBindingAPI(danger_prim)
        material_rel = mat_binding.GetDirectBinding().GetMaterial()
        if material_rel:
            shader_prim = stage.GetPrimAtPath(f"{material_rel.GetPath()}/Shader")
            if shader_prim.IsValid():
                shader_prim.GetAttribute("inputs:emissive_color").Set(Gf.Vec3f(r, g, b))
                shader_prim.GetAttribute("inputs:emissive_intensity").Set(intensity)

def safety_logic(min_dist):
    """거리에 따른 로봇 정지 및 LED 제어"""
    global is_robot_stopped
    if min_dist < 1.4:
        change_color(1.0, 0.0, 0.0) # 빨간색
        is_robot_stopped = True     # 로봇 정지 활성화
    elif min_dist < 2.2:
        change_color(1.0, 1.0, 0.0) # 노란색
        is_robot_stopped = False    # 주의 단계(감속 로직 등을 넣을 수 있음)
    else:
        change_color(0.0, 0.0, 1.0) # 파란색
        is_robot_stopped = False

def _find_end_effector_prim_path(stage, robot_prim_path):
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim.IsValid():
        return None
    preferred_names = ("ee_link", "tool0", "tool", "flange", "wrist_3_link")
    for prim in Usd.PrimRange(robot_prim):
        if prim.GetName() in preferred_names:
            return prim.GetPath().pathString
    last_link = None
    for prim in Usd.PrimRange(robot_prim):
        if "link" in prim.GetName():
            last_link = prim
    if last_link is not None:
        return last_link.GetPath().pathString
    return None

# --- 환경 구축 ---

if os.path.exists(usd_path):
    omni.usd.get_context().open_stage(usd_path)
else:
    print(f"Error: {usd_path} Not Found")
    simulation_app.close()
    exit()

my_world = World(stage_units_in_meters=1.0)
stage = my_world.stage
timeline = omni.timeline.get_timeline_interface()
lidarInterface = _range_sensor.acquire_lidar_sensor_interface()

# 1. 로봇 태스크 및 컨트롤러 설정
my_task = FollowTarget(
    name="ur10e_follow_target",
    target_position=np.array([0.5, 0, 0.5]),
    robot_prim_path=robot_path,
    attach_robot=True)
my_world.add_task(my_task)
my_world.reset()

task_params = my_world.get_task("ur10e_follow_target").get_params()
target_name = task_params["target_name"]["value"]
ur10e_name = task_params["robot_name"]["value"]
my_ur10e = my_world.scene.get_object(ur10e_name)

gripper = None
end_effector_prim_path = _find_end_effector_prim_path(stage, robot_path)
if end_effector_prim_path:
    gripper = SurfaceGripper(
        end_effector_prim_path=end_effector_prim_path,
        surface_gripper_path=f"{end_effector_prim_path}/SurfaceGripper",
    )
    if hasattr(my_ur10e, "gripper"):
        my_ur10e.gripper = gripper
else:
    print(f"Warning: end effector prim not found under {robot_path}, gripper not attached.")

my_controller = RMPFlowController(name="target_follower_controller", robot_articulation=my_ur10e)
articulation_controller = my_ur10e.get_articulation_controller()

# 2. LiDAR 및 세맨틱 설정
omni.kit.commands.execute("RangeSensorCreateLidar",    
    path="/LidarName", parent=robot_path, 
    min_range=0.4, max_range=20.0, draw_points=True, 
    horizontal_fov=360.0, vertical_fov=60.0, enable_semantics=True, rotation_rate = 0
)

human_prim = stage.GetPrimAtPath(human_path)
if human_prim.IsValid():
    initial_human_pos = human_prim.GetAttribute("xformOp:translate").Get()
    for prim in Usd.PrimRange(human_prim):
        if prim.IsA(UsdGeom.Mesh):
            add_update_semantics(prim=prim, semantic_label="human")

# --- 시뮬레이션 루프 ---

my_world.play()
frame_count = 0

try:
    while simulation_app.is_running():
        my_world.step(render=True)
        current_time = timeline.get_current_time()
        
        if my_world.is_playing():
            # LiDAR 감지 로직
            semantics = lidarInterface.get_prim_data(lidar_full_path)
            depth = lidarInterface.get_linear_depth_data(lidar_full_path)
            
            if len(semantics) > 0:
                semantics_np = np.array(semantics)
                human_indices = np.where(semantics_np == human_path)[0]
                if len(human_indices) > 0:
                    min_dist = np.min(np.array(depth)[human_indices])
                    safety_logic(min_dist)
                else:
                    change_color(0.0, 0.0, 1.0) # 사람 안 보이면 파랑
                    is_robot_stopped = False

            # 로봇 동작 제어
            if not is_robot_stopped:
                observations = my_world.get_observations()
                actions = my_controller.forward(
                    target_end_effector_position=observations[target_name]["position"],
                    target_end_effector_orientation=observations[target_name]["orientation"],
                )
                articulation_controller.apply_action(actions)
            else:
                # 로봇 정지: 모든 관절 속도를 0으로
                articulation_controller.apply_action(np.zeros(my_ur10e.num_dof))

            # 사람 이동 (사인 함수 왕복)
            if human_prim.IsValid():
                offset_x = 1.5 * math.sin(current_time * 0.1 * 2 * math.pi)
                new_pos = Gf.Vec3d(initial_human_pos[0] + offset_x, initial_human_pos[1], initial_human_pos[2])
                human_prim.GetAttribute("xformOp:translate").Set(new_pos)

except Exception as e:
    print(f"Simulation Error: {e}")
finally:
    if my_world is not None:
        my_world.stop()
    # 앱 종료 전 모든 이벤트를 처리할 시간을 줍니다.
    simulation_app.update() 
    simulation_app.close()
