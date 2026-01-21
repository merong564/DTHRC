import omni
import asyncio
from isaacsim.sensors.physx import _range_sensor
from pxr import UsdGeom, Gf
from omni.isaac.core.utils.semantics import add_update_semantics

stage = omni.usd.get_context().get_stage()
timeline = omni.timeline.get_timeline_interface()
lidarInterface = _range_sensor.acquire_lidar_sensor_interface()

# 1. 라이다 설정
lidar_full_path = "/World/LidarName"
if not stage.GetPrimAtPath('/World/PhysicsScene'):
    omni.kit.commands.execute('AddPhysicsSceneCommand', stage=stage, path='/World/PhysicsScene')

# 라이다 생성 (이미 있다면 생성 명령은 무시됨)
omni.kit.commands.execute("RangeSensorCreateLidar",    
    path=lidar_full_path, parent="/World", min_range=0.4, max_range=20.0,
    draw_points=True, horizontal_fov=360.0, vertical_fov=60.0,
    horizontal_resolution=0.4, vertical_resolution=0.4, enable_semantics=True
)

async def setup_simulation():
    # 시뮬레이션 시작 전 객체 확인
    human_path = "/World/male"
    
    # [수정된 33번째 줄 영역] 객체가 로드될 때까지 더 끈질기게 기다립니다.
    human_prim = stage.GetPrimAtPath(human_path)
    retry = 0
    while not human_prim.IsValid() and retry < 50:
        await asyncio.sleep(0.1)
        human_prim = stage.GetPrimAtPath(human_path)
        retry += 1

    if human_prim.IsValid():
        # 사람이 갑자기 이동하는 게 싫다면 아래 SetTranslate 좌표를 
        # 현재 Stage 창에 보이는 좌표로 수정하거나, 아예 주석 처리(#) 하세요.
        xform = UsdGeom.XformCommonAPI(human_prim)

        
        # 세맨틱 적용
        add_update_semantics(prim=human_prim, semantic_label="human")
        print(f"성공: {human_path} 설정 완료")
    else:
        print(f"실패: {human_path}를 찾을 수 없습니다.")
        return

    # 설정 완료 후 시뮬레이션 재생
    timeline.play()
    
    # 라이다 데이터 확인
    await asyncio.sleep(1.0)
    semantics = lidarInterface.get_semantic_data(lidar_full_path)
    if len(semantics) > 0:
        print("데이터 수집 중...")

asyncio.ensure_future(setup_simulation())
