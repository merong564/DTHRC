import numpy as np

from core.utils import bbox_lines_world, get_world_translation


_DEFAULT_CAMERA_RESOLUTION = (640, 480)


def _filter_detections_in_center(
    detections,
    center_ratio=1.0,
    fallback_resolution=_DEFAULT_CAMERA_RESOLUTION,
    edge_margin_px=80,
):
    if not detections:
        return []
    resolution = fallback_resolution
    if not resolution:
        return detections
    width, height = resolution
    margin = max(float(edge_margin_px), 0.0)
    edge_min_x = margin
    edge_max_x = width - margin
    if edge_max_x <= edge_min_x:
        edge_min_x = 0.0
        edge_max_x = float(width)
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
        if not (edge_min_x <= cx <= edge_max_x):
            continue
        if min_x <= cx <= max_x and min_y <= cy <= max_y:
            filtered.append(det)
    return filtered


def _collect_class_candidates(stage, class_names):   # detection된 클래스명과 stage내 prim 이름/경로 비교
    candidates = {class_name: [] for class_name in class_names}
    for prim in stage.Traverse():
        name = prim.GetName().lower()
        path = prim.GetPath().pathString.lower()
        for class_name in class_names:
            if class_name in name or class_name in path:
                candidates[class_name].append(prim)
    return candidates


def select_best_target_prim_path(
    stage,
    camera_sensor,
    detections,
    depth_m,
    base_prim_path=None,
    min_confidence=0.25,
    center_ratio=1.0,
):
    if not detections:
        return None, None
    filtered_dets = [
        d
        for d in detections
        if d.get("confidence", 0.0) >= min_confidence and d.get("class")
    ]
    filtered_dets = _filter_detections_in_center(
        filtered_dets, center_ratio=center_ratio
    )
    if not filtered_dets:
        return None, None

    class_names = {d.get("class", "").lower() for d in filtered_dets if d.get("class")}
    if not class_names:
        return None, None
    candidates_by_class = _collect_class_candidates(stage, class_names)

    base_position = None
    if base_prim_path:
        base_prim = stage.GetPrimAtPath(base_prim_path)
        if base_prim.IsValid():
            base_position = np.array(get_world_translation(base_prim), dtype=np.float64)

    best = None
    for det in filtered_dets:
        class_name = det.get("class", "").lower()
        if not class_name:
            continue
        candidates = candidates_by_class.get(class_name, [])
        if not candidates:
            continue

        center = np.array([det["center"]], dtype=np.float32)
        depth = np.array([depth_m], dtype=np.float32)
        world_point = camera_sensor.get_world_points_from_image_coords(center, depth)[0]

        best_prim = min(
            candidates,
            key=lambda p: np.linalg.norm(np.array(get_world_translation(p)) - world_point),
        )
        prim_position = np.array(get_world_translation(best_prim), dtype=np.float64)
        distance = 0.0
        if base_position is not None:
            distance = np.linalg.norm(prim_position - base_position)

        confidence = det.get("confidence", 0.0)
        if (
            best is None
            or confidence > best["confidence"]
            or (confidence == best["confidence"] and distance < best["distance"])
        ):
            best = {
                "prim_path": best_prim.GetPath().pathString,
                "class": class_name,
                "confidence": confidence,
                "distance": distance,
            }

    if best is None:
        return None, None
    return best["prim_path"], best["class"]


def select_target_prim_path(
    stage,
    camera_sensor,
    detections,
    depth_m,
    target_class="bolt",
    min_confidence=0.25,
    center_ratio=1.0,
):
    target_class = (target_class or "").lower()
    if not target_class:
        return None
    class_dets = [
        d
        for d in detections
        if d.get("class") == target_class and d.get("confidence", 0.0) >= min_confidence
    ]
    class_dets = _filter_detections_in_center(
        class_dets, center_ratio=center_ratio
    )
    if not class_dets:
        return None
    best_det = max(class_dets, key=lambda d: d.get("confidence", 0.0))
    center = np.array([best_det["center"]], dtype=np.float32)
    depth = np.array([depth_m], dtype=np.float32)
    world_point = camera_sensor.get_world_points_from_image_coords(center, depth)[0]
    candidates = []
    for prim in stage.Traverse():
        name = prim.GetName().lower()
        path = prim.GetPath().pathString.lower()
        if target_class in name or target_class in path:
            candidates.append(prim)
    if not candidates:
        return None
    best_prim = min(
        candidates,
        key=lambda p: np.linalg.norm(np.array(get_world_translation(p)) - world_point),
    )
    return best_prim.GetPath().pathString


def select_bolt_prim_path(
    stage,
    camera_sensor,
    detections,
    depth_m,
    min_confidence=0.25,
    center_ratio=1.0,
):
    return select_target_prim_path(
        stage,
        camera_sensor,
        detections,
        depth_m,
        target_class="bolt",
        min_confidence=min_confidence,
        center_ratio=center_ratio,
    )


def draw_detection_bboxes(debug_draw, camera_sensor, detections, depth_m):
    debug_draw.clear_lines()
    if not detections:
        return
    filtered_detections = _filter_detections_in_center(detections)
    if not filtered_detections:
        return
    line_starts = []
    line_ends = []
    colors = []
    sizes = []
    for res in filtered_detections:
        starts, ends = bbox_lines_world(camera_sensor, res["bbox"], depth_m)
        line_starts.extend(starts)
        line_ends.extend(ends)
        color = (0.0, 1.0, 0.0, 1.0) if res["class"] == "bolt" else (1.0, 0.6, 0.0, 1.0)
        colors.extend([color] * 4)
        sizes.extend([2.0] * 4)
    debug_draw.draw_lines(line_starts, line_ends, colors, sizes)
