import os
import torch
import numpy as np
from ultralytics import YOLO

class YoloDetector:
    def __init__(self):
        # 1. 현재 이 파일(yolo.py)이 있는 위치를 기준으로 경로 설정
        # core/yolo.py 위치를 기준으로 상위(..) -> model/ 폴더로 이동
        current_file_path = os.path.abspath(__file__) # yolo.py의 절대 경로
        core_dir = os.path.dirname(current_file_path) # core/ 폴더 경로
        project_root = os.path.dirname(core_dir)      # DTHRC/ 프로젝트 루트 경로
        
        # 프로젝트 루트를 기준으로 상대적인 모델 경로 설정
        model_path = os.path.join(
            project_root, 
            "model/BoltNut_Detection/finetuned_yolov8s_50_32_0.001/weights/best.pt"
        )
        
        # 2. 파일 존재 확인 (디버깅용)
        if not os.path.exists(model_path):
            print(f"\n❌ [ERROR] 모델을 찾을 수 없습니다!")
            print(f"계산된 경로: {model_path}")
            # 현재 작업 디렉토리 출력 (어디서 실행 중인지 확인)
            print(f"현재 작업 위치: {os.getcwd()}\n")
            raise FileNotFoundError(f"Model not found at: {model_path}")

        # 3. 모델 로드
        print(f"🚀 모델 로딩 중: {model_path}")
        self.model = YOLO(model_path)
        self.classes = ["bolt", "nut"]

    def preprocess(self, rgba_image):
        """Isaac Sim의 RGBA 이미지를 YOLO 입력용 RGB로 변환"""
        if rgba_image is None or rgba_image.size == 0:
            return None
        
        # RGBA -> RGB 채널 슬라이싱
        rgb = rgba_image[:, :, :3]
        
        # Isaac Sim 데이터가 float(0.0~1.0)인 경우 255를 곱함
        if rgb.dtype == np.float32 or rgb.dtype == np.float64:
            rgb = (rgb * 255).clip(0, 255).astype(np.uint8)
        else:
            rgb = rgb.astype(np.uint8)
            
        return rgb

    def detect(self, frame):
        """이미지 내 볼트와 너트 탐지 및 터미널 출력"""
        rgb_frame = self.preprocess(frame)
        if rgb_frame is None:
            return []

        # 추론 실행 (conf는 0.25 정도로 설정하여 가짜 인식을 방지)
        results = self.model.predict(rgb_frame, conf=0.25, verbose=False)
        
        detections = []
        for result in results:
            boxes = result.boxes
            for box in boxes:
                conf = box.conf[0].item()
                cls_idx = int(box.cls[0].item())
                label = self.classes[cls_idx]
                
                
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                detections.append({
                    "class": label,
                    "bbox": [x1, y1, x2, y2],
                    "center": [(x1 + x2) / 2, (y1 + y2) / 2],
                    "confidence": conf
                })
        
        if not detections:
            # 아무것도 탐지되지 않았을 때 (디버깅용)
            # print("... 탐지 중 ...")
            pass
            
        return detections

    def get_3d_coordinates(self, camera, pixel_coords, depth_data=None):
        """2D 픽셀을 3D 월드 좌표로 변환"""
        # 픽셀 좌표가 리스트 형태면 numpy 배열로 변환
        coords = np.array([pixel_coords])
        world_point = camera.get_world_points_from_image_coords(coords, depth_data)
        return world_point