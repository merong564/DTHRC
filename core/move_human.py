# 거리에 따라 사람 정지 및 LED 색상 변경 통합 코드
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

# --- 함수 정의 ---

def change_color(r, g, b, intensity=10000.0):
    """danger 객체의 Emissive 색상을 변경하는 함수"""
    if danger_prim.IsValid():
        mat_binding = UsdShade.MaterialBindingAPI(danger_prim)
        material_rel = mat_binding.GetDirectBinding().GetMaterial()
        
        if material_rel:
            shader_prim = stage.GetPrimAtPath(f"{material_rel.GetPath()}/Shader")
            if shader_prim.IsValid():
                shader_prim.GetAttribute("inputs:emissive_color").Set(Gf.Vec3f(r, g, b))
                shader_prim.GetAttribute("inputs:emissive_intensity").Set(intensity)

def safety_func(min_dist):
    """거리에 따른 경고등 색상 및 로직 처리"""
    if min_dist < 1.4:
        # 정지 상태: 빨간색
        change_color(1.0, 0.0, 0.0)
    elif min_dist < 2.2:
        # 감속 구간: 노란색
        change_color(1.0, 1.0, 0.0)
    else:
        # 정상 상태: 파란색
        change_color(0.0, 0.0, 1.0)

# --- 환경 설정 ---

usd_path = "/home/rokey/Desktop/DTHRC/env_default.usd"

if os.path.exists(usd_path):
    omni.usd.get_context().open_stage(usd_path)
else:
    print(f"Error: Cannot find USD file at {usd_path}")
    simulation_app.close()
    exit()

stage = omni.usd.get_context().get_stage()
timeline = omni.timeline.get_timeline_interface()
lidarInterface = _range_sensor.acquire_lidar_sensor_interface()

robot_path = "/World/ur10e"
lidar_full_path = f"{robot_path}/LidarName"
human_path = "/World/male"
danger_path = "/World/danger"

# 프림 가져오기
human_prim = stage.GetPrimAtPath(human_path)
danger_prim = stage.GetPrimAtPath(danger_path)

# 사람 초기 위치 저장 (왕복 기준점)
if human_prim.IsValid():
    initial_human_pos = human_prim.GetAttribute("xformOp:translate").Get()
else:
    initial_human_pos = Gf.Vec3d(0, 0, 0)

# 왕복 운동 변수
amplitude = 1.5  # 1m 범위
frequency = 0.1  # 속도


# 물리 및 라이다 설정 (기본 제공 코드 유지)
if not stage.GetPrimAtPath('/World/PhysicsScene'):
    omni.kit.commands.execute('AddPhysicsSceneCommand', stage=stage, path='/World/PhysicsScene')

omni.kit.commands.execute("RangeSensorCreateLidar",    
    path="/LidarName", parent=robot_path, 
    min_range=0.4, max_range=20.0, draw_points=True, 
    horizontal_fov=360.0, vertical_fov=60.0, enable_semantics=True, rotation_rate = 0,
    horizontal_resolution = 1.0, vertical_resolution = 1.0
)

# 세맨틱 설정
if human_prim.IsValid():
    for prim in Usd.PrimRange(human_prim):
        if prim.IsA(UsdGeom.Mesh):
            add_update_semantics(prim=prim, semantic_label="human")

# --- 시뮬레이션 루프 ---

timeline.play()
frame_count = 0

try:
    while simulation_app.is_running():
        simulation_app.update()
        current_time = timeline.get_current_time()
        
        if frame_count > 60:
            semantics = lidarInterface.get_prim_data(lidar_full_path)
            depth = lidarInterface.get_linear_depth_data(lidar_full_path)
            
            if len(semantics) > 0:
                semantics_np = np.array(semantics)
                depth_np = np.array(depth)
                human_indices = np.where(semantics_np == human_path)[0]
                
                if len(human_indices) > 0:
                    min_dist = np.min(depth_np[human_indices])
                    # 거리에 따른 색상 변경 및 이동 여부 결정
                    safety_func(min_dist)
                else:
                    # is_human_moving = True # 안 보이면 이동
                    change_color(0.0, 0.0, 1.0) # 기본 파란색

            # 사람 이동 로직 (사인 함수 왕복)
            if human_prim.IsValid():
                offset_x = amplitude * math.sin(current_time * frequency * 2 * math.pi)
                new_pos = Gf.Vec3d(initial_human_pos[0] + offset_x, initial_human_pos[1], initial_human_pos[2])
                human_prim.GetAttribute("xformOp:translate").Set(new_pos)
            
        frame_count += 1

except Exception as e:
    print(f"Error: {e}")
finally:
    timeline.stop()
    simulation_app.close()