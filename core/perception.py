import numpy as np

from core.utils import bbox_lines_world, get_world_translation


_DEFAULT_CAMERA_RESOLUTION = (640, 480)


def _filter_detections_in_center(
    detections,
    center_ratio=0.5,
    fallback_resolution=_DEFAULT_CAMERA_RESOLUTION,
):
    if not detections:
        return []
    resolution = fallback_resolution
    if not resolution:
        return detections
    width, height = resolution
    ratio = min(max(center_ratio, 0.0), 1.0)
    half_w = width * ratio / 2.0
    half_h = height * ratio / 2.0
    center_x = width / 2.0
    center_y = height / 2.0
    min_x = center_x - half_w
    max_x = center_x + half_w
    min_y = center_y - half_h
    max_y = center_y + half_h

    filtered = []
    for det in detections:
        center = det.get("center")
        if not center or len(center) < 2:
            continue
        cx, cy = center[0], center[1]
        if min_x <= cx <= max_x and min_y <= cy <= max_y:
            filtered.append(det)
    return filtered


def select_bolt_prim_path(
    stage,
    camera_sensor,
    detections,
    depth_m,
    min_confidence=0.25,
    center_ratio=0.5,
):
    bolt_dets = [
        d
        for d in detections
        if d.get("class") == "bolt" and d.get("confidence", 0.0) >= min_confidence # 임계값보다 정확도 높은 볼트 검출 결과만 선택
    ]
    bolt_dets = _filter_detections_in_center(bolt_dets, center_ratio=center_ratio) # 화면 중앙에 있는 볼트 검출 결과만 선택
    if not bolt_dets:
        return None
    best_det = max(bolt_dets, key=lambda d: d.get("confidence", 0.0))   # 가장 높은 정확도의 볼트 검출 결과 선택
    center = np.array([best_det["center"]], dtype=np.float32)
    depth = np.array([depth_m], dtype=np.float32)
    world_point = camera_sensor.get_world_points_from_image_coords(center, depth)[0]
    candidates = []
    for prim in stage.Traverse():
        name = prim.GetName().lower()
        path = prim.GetPath().pathString.lower()
        if "bolt" in name or "bolt" in path:
            candidates.append(prim)
    if not candidates:
        return None
    best_prim = min(
        candidates,
        key=lambda p: np.linalg.norm(np.array(get_world_translation(p)) - world_point),
    )
    return best_prim.GetPath().pathString


def draw_detection_bboxes(debug_draw, camera_sensor, detections, depth_m):
    debug_draw.clear_lines()
    if not detections:
        return
    line_starts = []
    line_ends = []
    colors = []
    sizes = []
    for res in detections:
        starts, ends = bbox_lines_world(camera_sensor, res["bbox"], depth_m)
        line_starts.extend(starts)
        line_ends.extend(ends)
        color = (0.0, 1.0, 0.0, 1.0) if res["class"] == "bolt" else (1.0, 0.6, 0.0, 1.0)
        colors.extend([color] * 4)
        sizes.extend([2.0] * 4)
    debug_draw.draw_lines(line_starts, line_ends, colors, sizes)
