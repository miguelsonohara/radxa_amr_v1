from __future__ import annotations

import math
from dataclasses import dataclass

from .types import LidarCluster, LidarPoint2D


@dataclass(frozen=True)
class ClusterFilterParams:
    n_min: int
    width_min: float
    width_max: float
    range_min: float
    range_max: float


def _distance(a: LidarPoint2D, b: LidarPoint2D) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def _build_cluster(points: list[LidarPoint2D]) -> LidarCluster:
    xs = [p.x for p in points]
    ys = [p.y for p in points]
    rs = [math.hypot(p.x, p.y) for p in points]
    width = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
    return LidarCluster(
        points=points,
        centroid_x=sum(xs) / len(xs),
        centroid_y=sum(ys) / len(ys),
        frame_id=points[0].frame_id,
        n_points=len(points),
        width=width,
        mean_range=sum(rs) / len(rs),
    )


def cluster_points(points: list[LidarPoint2D], eps: float) -> list[LidarCluster]:
    if not points:
        return []
    ordered = sorted(points, key=lambda p: math.atan2(p.y, p.x))
    groups: list[list[LidarPoint2D]] = [[ordered[0]]]
    for pt in ordered[1:]:
        if _distance(groups[-1][-1], pt) <= eps:
            groups[-1].append(pt)
        else:
            groups.append([pt])
    return [_build_cluster(g) for g in groups if g]


def filter_human_clusters(clusters: list[LidarCluster], params: ClusterFilterParams) -> list[LidarCluster]:
    out: list[LidarCluster] = []
    for c in clusters:
        if c.n_points < params.n_min:
            continue
        if c.width < params.width_min or c.width > params.width_max:
            continue
        if c.mean_range < params.range_min or c.mean_range > params.range_max:
            continue
        out.append(c)
    return out


def select_phase1_cluster(clusters: list[LidarCluster]) -> tuple[LidarCluster | None, str]:
    if len(clusters) == 0:
        return None, "empty"
    if len(clusters) == 1:
        return clusters[0], "ok"
    return None, "ambiguous"
