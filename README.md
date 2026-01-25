# DTHRC (Digital Twin & Hybrid Robot Control)
# 인간-로봇 협업 디지털 트윈 시뮬레이션

- ROKEY 부트캠프 6기 E1조 '심봤다' 팀의 프로젝트 레포지토리입니다.

# 1. 프로젝트 개요
- 개발 목적: 산업 현장에서 사람과 로봇이 공존할 때 발생하는 안전 리스크를 디지털 트윈(Isaac Sim)에서 재현하고, YOLOv8 인식 및 RMPflow 경로 최적화를 통해 로봇의 안전 대응(감속/정지)과 작업 효율(부품 전달)을 동시에 확보함.

- 개발 기간: 2026.01.19 ~ 2026.01.26 (7일)

# 2. Team Members
  
| **곽문정** | **진재협** | **이주노** | **지승아** | **이채영**|
|:------:|:------:|:------:|:------:|:------:
| <img src="https://github.com/user-attachments/assets/86b2f0a0-4f78-4295-b312-8b93bfe75287" alt="곽문정" width="150"> | <img src="https://github.com/user-attachments/assets/86b2f0a0-4f78-4295-b312-8b93bfe75287" alt="진재협" width="150"> | <img src="https://github.com/user-attachments/assets/86b2f0a0-4f78-4295-b312-8b93bfe75287" alt="이주노" width="150"> | <img src="https://github.com/user-attachments/assets/86b2f0a0-4f78-4295-b312-8b93bfe75287" alt="지승아" width="150"> |<img src="https://github.com/user-attachments/assets/86b2f0a0-4f78-4295-b312-8b93bfe75287" alt="이채" width="150"> |
| 프로젝트 총괄 | FE | BE | AI | 비전인 |
| [GitHub](https://github.com/merong564) | [GitHub](https://github.com/yichaeyoung) | [GitHub](https://github.com/yichaeyoung) | [GitHub](https://github.com/yichaeyoung) | [GitHub](https://github.com/yichaeyoung) |


# 3. 실행 가이드

- 실행 환경 세팅

```isaacsim
mkdir ~/isaacsim
cd ~/Downloads
unzip "isaac-sim-standalone-5.0.0-linux-x86_64.zip" -d ~/isaacsim
cd ~/isaacsim
./post_install.sh
./isaac-sim.sh # 아이작심 실행 코드
```

- yolo 세팅

```ultralytics
cd ~/isaacsim
./python.sh -m pip install ultralytics
./python.sh -m pip install opencv-python
./python.sh -m pip install supervision
./python.sh -m pip install "numpy==1.26.0" #Isaac Sim 확장/바이너리 호환 때문에 NumPy 2.x 계열에서 충돌 가능성이 커서 고정.
```

# 4. 폴더 구조

```plaintext
project/
├── assets/
├── core/
│   ├── bolt_nut.py                 # Bolt, Nut 생성
│   ├── controller.py               # 삭제 예정
│   ├── env.py                      # isaac sim asset 추가 
│   ├── move.py                     # Human asset 좌표 이동
│   ├── perception.py               # Camera와 가까운 물체에 Bounding box 생성 
│   ├── pick_and_place.py           # robot Pick & Place 자동화 로직
│   ├── pick_and_place_follow.py    # ...
│   ├── pick_and_place_gripper.py   # ...
│   ├── safety.py                   # LiDAR Semantics 기반 3단계 안전 존(Blue/Yellow/Red) 제어 로직
│   ├── sensor.py                   # LiDAR 및 Camera 생성
│   ├── utils.py                    # ... 여러가지
│   └── yolo.py                     # YOLOv8 모델 로드 및 실시간 객체 탐지
├── utils/                          # 시뮬레이션 내 여러 구동 파일
├── .gitignore
├── README.md                # 프로젝트 개요 및 사용법
└── main.py                  # 전체 Code 실행 파일
```

# 5. 주요 기능

### 5.1  Motion Control (RMPflow & FSM)
- 유연한 경로 생성: RMPflow를 통해 타겟을 실시간으로 추적.
  
- FSM 공정: 접근 → 파지(Grasp) → 상승 → 배치 → 해제로 이어지는 상태기계 기반의 안정적인 Pick & Place.

### 5.2 Perception (YOLOv8s)
- 부품 정밀 인식: 볼트(Bolt) 및 너트(Nut)를 92.7% 이상의 mAP50으로 실시간 탐지.

- Sim2Real: 시뮬레이션 데이터를 실제 환경 데이터와 통합하여 환경 변화에 강인한 모델 구축.

### 5.3 Safety Stack
- LiDAR Semantics: 사람(Human) 객체를 필터링하여 최소 거리를 실시간 계산 후 안.

- 가변 대응: 거리에 따라 BLUE(안전), YELLOW(감속), RED(정지/LED 알람) 3단계 안전 시스템 가동.

# 6. 기술 스택 및 개발 환경

<div style="display:flex; flex-direction:row;">
  <img src="https://img.shields.io/badge/ubuntu-E95420?style=flat&logo=Python&logoColor=white" />
  <img src="https://img.shields.io/badge/Python-3776AB?style=flat&logo=Python&logoColor=white" />
  <img src="https://img.shields.io/badge/nvidia-76B900?style=flat&logo=nvidia&logoColor=white" />
  <img src="https://img.shields.io/badge/yolo-111F68?style=flat&logo=nvidia&logoColor=white" />
  <img src="https://img.shields.io/badge/github-%23181717.svg?&style=flaat&logo=github&logoColor=white" />
  <img src="https://img.shields.io/badge/notion-%23000000.svg?&style=flat&logo=notion&logoColor=white" />
</div><br>

# 7. 아키텍쳐
1. Input: LiDAR 센서 및 가상 카메라 데이터 입력.

2. Process:

    - core/safety.py에서 작업자 거리 분석.

    - core/yolo.py에서 작업 대상물 위치 파악.

    - utils/rmpflow_controller.py에서 최적 이동 경로 계산.

3. Output: 로봇 관절 속도 제어 및 Surface Gripper 동작 수행.
