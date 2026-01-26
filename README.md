# DTHRC (Digital Twin & Hybrid Robot Control)
# 인간-로봇 협업 디지털 트윈 시뮬레이션

- ROKEY 부트캠프 6기 E1조 '심봤다' 팀의 프로젝트 레포지토리입니다.

# 1. 프로젝트 개요
### 1.1 개발 목적
 - 산업 현장에서 사람과 로봇이 협업할 때 발생할 수 있는 안전 리스크를
   디지털 트윈 환경(Isaac Sim)에서 재현하고,

   **YOLOv8 기반 객체 인식**

   **RMPflow 기반 경로 최적화**

    를 결합하여
    로봇의 **안전 대응(감속/정지)** 과 **작업 효율(부품 전달)** 을 동시에 확보하는 것을 목표로 합니다.

### 1.2 개발 기간
 - 2026.01.16 ~ 2026.01.26 (10일)

# 2. Team Members
  
| **곽문정** | **지승아** | **이주노** | **진재협** | **이채영**|
|:------:|:------:|:------:|:------:|:------:
| <img src="https://github.com/user-attachments/assets/ab576294-86aa-4364-bf40-13cbf76a426a" alt="곽문정" width="150"> | <img src="https://github.com/user-attachments/assets/41ed25b8-1832-4552-9f2e-cc96acb3bfee" alt="지승아" width="150"> | <img src="https://github.com/user-attachments/assets/d0da2053-ebf6-42c3-86e2-828177631ba2" alt="이주노" width="150"> | <img src="https://github.com/user-attachments/assets/8db457f6-efca-4a0c-898a-6973ecef9f78" alt="진재협" width="150"> |<img src="https://github.com/user-attachments/assets/bc5b8629-ec6a-4cae-b70f-1c8449b206e7" alt="이채영" width="150"> |
| PL | pick & place | safety | YOLO | YOLO |
| [GitHub](https://github.com/merong564) | [GitHub](https://github.com/seounga) | [GitHub](https://github.com/dlwnsh925) | [GitHub](https://github.com/comport98) | [GitHub](https://github.com/yichaeyoung) |


# 3. 실행 가이드

### 3.1 Isaac sim 설치

```isaacsim
mkdir ~/isaacsim
cd ~/Downloads
unzip "isaac-sim-standalone-5.0.0-linux-x86_64.zip" -d ~/isaacsim
cd ~/isaacsim
./post_install.sh
```

### 3.2 yolo 환경 설정

```ultralytics
cd ~/isaacsim
./python.sh -m pip install ultralytics
./python.sh -m pip install opencv-python
./python.sh -m pip install supervision
./python.sh -m pip install "numpy==1.26.0" #Isaac Sim 확장/바이너리 호환 때문에 NumPy 2.x 계열에서 충돌 가능성이 커서 고정.
```

### 3.3 프로젝트 실행
Isaac Sim 확장 및 물리 엔진과 연동된 Python 환경에서 프로젝트를 실행합니다.

```bash
git clone -b develop https://github.com/merong564/DTHRC.git
cd ~/Desktop/DTHRC/DTHRC
./python.sh /home/rokey/Desktop/DTHRC/DTHRC/main.py
```

# 4. 폴더 구조

```plaintext
project/
├── assets/
├── core/
│   ├── bolt_nut.py                 # Bolt, Nut 객체 정의
│   ├── env.py                      # 월드 초기화, 로봇 로딩, 오브젝트 관리
│   ├── move.py                     # Human Asset 이동 제어
│   ├── perception.py               # Bounding Box 생성
│   ├── pick_and_place_gripper.py   # Pick & Place 자동화 로직
│   ├── safety.py                   # LiDAR 기반 3단계 안전 존 제어
│   ├── sensor.py                   # LiDAR 및 Camera 생성
│   ├── utils.py                    # 시각화 및 좌표 변환 유틸
│   └── yolo.py                     # YOLOv8 객체 탐지
├── model/                          # Bolt·Nut 학습 모델(best.pt)
├── utils/
│   └── rmpflow_controller.py       # RMPflow 모션 컨트롤러
├── main.py                         # 전체 시뮬레이션 실행 엔트리
└── README.md
```

# 5. 주요 기능

### 5.1  Motion Control (RMPflow & FSM)
 - RMPflow 기반 실시간 경로 생성
 - Approach → Grasp → Lift → Place → Release → Return
   
    FSM 기반 Pick & Place 공정 제어

### 5.2 Perception (YOLOv8s)
 - 볼트(Bolt), 너트(Nut) 실시간 객체 인식
 - mAP50 ≥ 92.7%
 - Sim2Real 데이터 확장을 통한 환경 변화 대응

### 5.3 Safety Stack
 - LiDAR Semantics 기반 작업자 인식
 - 거리 기반 3단계 안전 제어
  - BLUE: 정상 동작
  - YELLOW: 감속
  - RED: 정지

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
### 7.1 Input
 - LiDAR 센서: 작업자(Human)와 로봇 간 거리 측정
 - Camera 센서: 작업 대상물(Bolt / Nut) 이미지 입력

### 7.2 Process
  - core/safety.py : 작업자 거리 분석
  - core/yolo.py : 작업 대상물 인식
  - utils/rmpflow_controller.py : 최적 경로 계산
  - core/pick_and_place.py : FSM 기반 작업 제어를 수행 (Approach → Grasp → Lift → Place → Release → Return)
    - 한 사이클이 종료되면 자동으로 객체 전환

### 7.3 Output
  - 로봇 관절 속도 제어
  - Surface Gripper 제어
