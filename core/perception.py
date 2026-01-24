import numpy as np

from core.utils import bbox_lines_world, get_world_translation


def select_bolt_prim_path(stage, camera_sensor, detections, depth_m):
    bolt_dets = [d for d in detections if d.get("class") == "bolt"]
    if not bolt_dets:
        return None
    best_det = max(bolt_dets, key=lambda d: d.get("confidence", 0.0))
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
