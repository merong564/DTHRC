import omni
import asyncio
from isaacsim.sensors.physx import _range_sensor
from pxr import UsdGeom, Gf
from omni.isaac.core.utils.semantics import add_update_semantics

stage = omni.usd.get_context().get_stage()
timeline = omni.timeline.get_timeline_interface()
lidarInterface = _range_sensor.acquire_lidar_sensor_interface()

# 1. 경로 설정
robot_path = "/World/ur10e"
# [수정] 라이다를 로봇의 자식으로 설정하기 위한 경로 구성
lidar_name = "LidarName"
lidar_full_path = f"{robot_path}/{lidar_name}" 

if not stage.GetPrimAtPath('/World/PhysicsScene'):
    omni.kit.commands.execute('AddPhysicsSceneCommand', stage=stage, path='/World/PhysicsScene')

# 2. 라이다 생성 (parent를 robot_path로 지정)
omni.kit.commands.execute("RangeSensorCreateLidar",    
    path=f"/{lidar_name}",      # parent를 지정할 때는 상대 경로처럼 이름만 씁니다.
    parent=robot_path,          # [수정] 로봇을 부모로 설정
    min_range=0.4, max_range=100.0,
    draw_points=True, horizontal_fov=360.0, vertical_fov=60.0,
    horizontal_resolution=0.4, vertical_resolution=0.4, enable_semantics=True
)

# [수정] 로봇 기준 상대 좌표 (0, 0, 0.1) 설정
lidar_prim = stage.GetPrimAtPath(lidar_full_path)
if lidar_prim:
    UsdGeom.XformCommonAPI(lidar_prim).SetTranslate((0.0, 0.0, 0.1))

async def setup_simulation():
    human_path = "/World/male"
    
    # 객체 로드 대기
    retry = 0
    human_prim = stage.GetPrimAtPath(human_path)
    while not human_prim.IsValid() and retry < 50:
        await asyncio.sleep(0.1)
        human_prim = stage.GetPrimAtPath(human_path)
        retry += 1

    if human_prim.IsValid():
        # 세맨틱 적용 (Isaac Sim 5.0 사양: type="class")
        add_update_semantics(prim=human_prim, semantic_label="human")
        print(f"성공: {human_path} 설정 및 세맨틱 적용 완료")
    else:
        print(f"실패: {human_path}를 찾을 수 없습니다.")
        return

    # 설정 완료 후 시뮬레이션 재생
    timeline.play()
    
    # 라이다 데이터 확인
    await omni.kit.app.get_app().next_update_async()
    
    # [수정] 데이터 호출 시 정확한 절대 경로 사용
    semantics = lidarInterface.get_semantic_data(lidar_full_path)
    if len(semantics) > 0:
        print("if")
    	# [추가] 라이다 데이터 확인 및 human 거리 계산
        await asyncio.sleep(1.0)
    
    # 1. 데이터 가져오기
        semantics = lidarInterface.get_semantic_data(lidar_full_path)
        depth = lidarInterface.get_linear_depth_data(lidar_full_path)

        if len(semantics) > 0 and len(depth) > 0:
            print("if")
            # 2. 'human' 레이블에 해당하는 ID 확인
            # Isaac Sim은 세맨틱 라벨을 내부적으로 정수 ID로 관리합니다.
            # "human"이라는 클래스 이름에 매핑된 ID를 가져옵니다.
            try:
                # get_semantic_class_label_id는 특정 라벨의 ID를 반환합니다.
                human_id = lidarInterface.get_semantic_class_label_id(lidar_full_path, "human")
                
                # 3. human ID와 일치하는 데이터 필터링 (numpy 활용 시 더 빠름)
                import numpy as np
                semantics_np = np.array(semantics)
                depth_np = np.array(depth)
                
                # human 객체에 부딪힌 포인트들의 인덱스 추출
                human_indices = np.where(semantics_np == human_id)[0]
                
                if len(human_indices) > 0:
                    # 4. 해당 인덱스의 거리값만 추출
                    human_distances = depth_np[human_indices]
                    
                    # 유효한 거리값(0보다 큰 값)만 필터링
                    valid_distances = human_distances[human_distances > 0]
                    
                    if len(valid_distances) > 0:
                        min_dist = np.min(valid_distances)
                        print(min_dist)
                        avg_dist = np.mean(valid_distances)
                        
                        print(f"[Safety System] Human 감지!")
                        print(f" - 최단 거리: {min_dist:.2f} m")
                        print(f" - 평균 거리: {avg_dist:.2f} m")
                        print(f" - 감지된 포인트 수: {len(valid_distances)} 개")
                    else:
                        print("Human이 범위 내에 있지만 유효한 거리 데이터가 없습니다.")
                else:
                    print("현재 라이다 시야 내에 Human이 없습니다.")
                    
            except Exception as e:
                print(f"세맨틱 ID 조회 중 오류 발생: {e}")
        else:
            print("라이다 데이터를 찾을 수 없습니다.")
            

asyncio.ensure_future(setup_simulation())