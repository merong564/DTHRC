import numpy as np
from pxr import UsdGeom


def _bbox_corners_xyxy(bbox):
    x1, y1, x2, y2 = bbox
    return np.array(
        [
            [x1, y1],
            [x2, y1],
            [x2, y2],
            [x1, y2],
        ],
        dtype=np.float32,
    )


def bbox_lines_world(camera_sensor, bbox, depth_m):
    points_2d = _bbox_corners_xyxy(bbox)
    depth = np.full((4,), depth_m, dtype=np.float32)
    points_3d = camera_sensor.get_world_points_from_image_coords(points_2d, depth)
    edges = [(0, 1), (1, 2), (2, 3), (3, 0)]
    starts = [tuple(points_3d[i]) for i, _ in edges]
    ends = [tuple(points_3d[j]) for _, j in edges]
    return starts, ends


def find_bolt_prim(stage):
    for prim in stage.Traverse():
        name = prim.GetName().lower()
        path = prim.GetPath().pathString.lower()
        if "bolt" in name or "bolt" in path:
            return prim
    return None


def get_world_translation(prim):
    xform_cache = UsdGeom.XformCache()
    transform = xform_cache.GetLocalToWorldTransform(prim)
    return transform.ExtractTranslation()
