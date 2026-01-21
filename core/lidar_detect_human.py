from omni.isaac.kit import SimulationApp
import os

# 1. 시뮬레이션 앱 초기화
simulation_app = SimulationApp({"headless": False}) 

import omni
import asyncio
import numpy as np
from isaacsim.sensors.physx import _range_sensor
from pxr import UsdGeom, Gf, Usd, Semantics, UsdShade
from omni.isaac.core.utils.semantics import add_update_semantics

def change_color(r,g,b):
    if danger_prim.IsValid():
    # Material 바인딩 인터페이스 가져오기
            mat_binding = UsdShade.MaterialBindingAPI(danger_prim)
            material_rel = mat_binding.GetDirectBinding().GetMaterial()
            
            if material_rel:
                # 2. 재질 내부의 Shader(Shader) 프림 찾기 (보통 'Shader'라는 이름임)
                shader_prim = stage.GetPrimAtPath(f"{material_rel.GetPath()}/Shader")
                
                if shader_prim.IsValid():
                    # 3. Emissive 속성 수정
                    # 색상을 노랑색으로 변경 (RGB)
                    shader_prim.GetAttribute("inputs:emissive_color").Set(Gf.Vec3f(r, g, b))
                    # 발광 강도 조절 (예: 1000.0으로 높여서 밝게 만들기)
                    shader_prim.GetAttribute("inputs:emissive_intensity").Set(100000000.0)
                    
                    print(f"Success: {danger_path}의 Emissive 속성을 변경했습니다.")

def safety_func(min_dist):
    """
    min_dist에 따라 경고등 색상을 변경하는 함수
    """
    if min_dist < 1.4:
        print("stop")
        # 위험 상태: 빨간색
        change_color(1.0, 0.0, 0.0)
    elif min_dist < 1.8:
        print("slow")
        change_color(1.0, 1.0, 0.0)
    else:
        print("normal")
        change_color(0.0, 0.0, 1.0)


# [추가] USD 파일 경로 설정
# 파일이 lidar_test.py와 같은 폴더에 있다면 아래와 같이 경로를 잡습니다.
usd_path = "/home/rokey/Desktop/DTHRC/env_default.usd"

# [추가] 스테이지 열기
if os.path.exists(usd_path):
    omni.usd.get_context().open_stage(usd_path)
    print(f"Successfully opened stage: {usd_path}")
else:
    print(f"Error: Cannot find USD file at {usd_path}")
    simulation_app.close()
    exit()

# 2. 기본 설정 (스테이지를 연 후에 가져와야 합니다)
stage = omni.usd.get_context().get_stage()
timeline = omni.timeline.get_timeline_interface()
lidarInterface = _range_sensor.acquire_lidar_sensor_interface()

robot_path = "/World/ur10e"
lidar_name = "LidarName"
lidar_full_path = f"{robot_path}/{lidar_name}"
human_path = "/World/male"
danger_path = "/World/danger"

# 3. 물리 씬 및 라이다 생성
if not stage.GetPrimAtPath('/World/PhysicsScene'):
    omni.kit.commands.execute('AddPhysicsSceneCommand', stage=stage, path='/World/PhysicsScene')

omni.kit.commands.execute("RangeSensorCreateLidar",    
    path=f"/{lidar_name}", 
    parent=robot_path, 
    min_range=0.4, max_range=20.0,
    draw_lines=True, horizontal_fov=360.0, vertical_fov=60.0,
    horizontal_resolution=0.4, vertical_resolution=0.4, enable_semantics=True, rotation_rate = 0
)

lidar_prim = stage.GetPrimAtPath(lidar_full_path)
danger_prim = stage.GetPrimAtPath(danger_path)

if lidar_prim:
    UsdGeom.XformCommonAPI(lidar_prim).SetTranslate((0.0, 0.0, 0.1))

# 4. Human 세맨틱 설정 (파일을 불러온 후 존재 여부 확인)
human_prim = stage.GetPrimAtPath(human_path)
if human_prim.IsValid():
    for prim in Usd.PrimRange(human_prim):
        if prim.IsA(UsdGeom.Mesh):
            add_update_semantics(prim=prim, semantic_label="human")
    print("Human semantic setup complete.")
else:
    print(f"Warning: Human prim at {human_path} not found in the loaded USD.")

# 5. 시뮬레이션 루프
timeline.play()
print("Simulation started...")

frame_count = 0
try:
    while simulation_app.is_running():
        simulation_app.update()
        
        if frame_count > 60:
            semantics = lidarInterface.get_prim_data(lidar_full_path)
            pointcloud = lidarInterface.get_point_cloud_data(lidar_full_path)
            depth = lidarInterface.get_linear_depth_data(lidar_full_path)
            print(semantics)
            
            if len(semantics) > 0:
                # human_id = lidarInterface.get_semantic_class_label_id(lidar_full_path, "human")
                semantics_np = np.array(semantics)
                depth_np = np.array(depth)
                human_indices = np.where(semantics_np == human_path)[0]
            
                
                if len(human_indices) > 0:
                    human_distances = depth_np[human_indices]
                    print(f"[Detected] Human Points: {len(human_indices)}")
                    print(f"First Human Point: {pointcloud[human_indices[0]]}")
                    min_dist = np.min(human_distances)
                    print(f"min_dist:{min_dist}")
                    safety_func(min_dist)
                else:
                    print("Human not in Lidar FOV.")
            
        frame_count += 1

except Exception as e:
    print(f"Error during simulation: {e}")
finally:
    timeline.stop()
    simulation_app.close()

