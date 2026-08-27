from __future__ import annotations

import math
from typing import Callable, Optional

from .types import LidarPoint2D


def _sanitize_range(r: float, range_min: float, range_max: float) -> bool:
    return math.isfinite(r) and range_min <= r <= range_max


def laser_scan_to_points(
    scan,
    frame_id: str = "laser",
    max_range: Optional[float] = None,
) -> list[LidarPoint2D]:
    ranges = getattr(scan, "ranges", [])
    angle_min = float(getattr(scan, "angle_min", 0.0))
    angle_increment = float(getattr(scan, "angle_increment", 0.0))
    range_min = float(getattr(scan, "range_min", 0.0))
    range_max = float(getattr(scan, "range_max", float("inf")))
    if max_range is not None:
        range_max = min(range_max, max_range)
    stamp_sec = _to_stamp_sec(scan)

    pts: list[LidarPoint2D] = []
    for i, r in enumerate(ranges):
        r_f = float(r)
        if not _sanitize_range(r_f, range_min, range_max):
            continue
        theta = angle_min + i * angle_increment
        pts.append(
            LidarPoint2D(
                x=r_f * math.cos(theta),
                y=r_f * math.sin(theta),
                frame_id=frame_id,
                theta=theta,
                r=r_f,
                stamp_sec=stamp_sec,
            )
        )
    return pts


def transform_points_to_map(
    points: list[LidarPoint2D],
    transform_fn: Callable[[float, float], tuple[float, float]] | None,
    out_frame_id: str = "map",
) -> list[LidarPoint2D]:
    if transform_fn is None:
        return []
    out: list[LidarPoint2D] = []
    for pt in points:
        x_map, y_map = transform_fn(pt.x, pt.y)
        out.append(
            LidarPoint2D(
                x=float(x_map),
                y=float(y_map),
                frame_id=out_frame_id,
                theta=pt.theta,
                r=pt.r,
                stamp_sec=pt.stamp_sec,
            )
        )
    return out


def _to_stamp_sec(scan) -> Optional[float]:
    header = getattr(scan, "header", None)
    stamp = getattr(header, "stamp", None) if header is not None else None
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if sec is None or nanosec is None:
        return None
    return float(sec) + float(nanosec) * 1e-9
