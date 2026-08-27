from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from .types import LidarPoint2D, ServiceZone


def load_zone_from_dict(data: dict[str, Any]) -> ServiceZone:
    return ServiceZone(
        frame_id=str(data.get("frame_id", "map")),
        x_min=float(data["x_min"]),
        y_min=float(data["y_min"]),
        x_max=float(data["x_max"]),
        y_max=float(data["y_max"]),
        epsilon=float(data.get("epsilon", 0.0)),
    )


def load_zone_from_yaml(path: str | Path) -> ServiceZone:
    with Path(path).open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    zone_data = data.get("service_zone", data)
    return load_zone_from_dict(zone_data)


def filter_points_in_zone(points: list[LidarPoint2D], zone: ServiceZone) -> list[LidarPoint2D]:
    return [pt for pt in points if zone.contains(pt.x, pt.y)]
