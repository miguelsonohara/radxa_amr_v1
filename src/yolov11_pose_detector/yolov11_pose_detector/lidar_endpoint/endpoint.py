from __future__ import annotations

import math
from collections import deque
from typing import Callable, Iterable, Optional

from .cluster import ClusterFilterParams, cluster_points, filter_human_clusters, select_phase1_cluster
from .scan_utils import laser_scan_to_points, transform_points_to_map
from .types import AlreadyThere, GoalPose, PersonEndpoint, Phase1Params, ServiceZone, TriggerEvent
from .zone import filter_points_in_zone


def compute_safety_goal(
    person_x: float,
    person_y: float,
    robot_x: float,
    robot_y: float,
    safety_distance: float,
    frame_id: str = "map",
    eps: float = 0.1,
) -> GoalPose | AlreadyThere:
    dx = person_x - robot_x
    dy = person_y - robot_y
    d = math.hypot(dx, dy)
    if d <= safety_distance + eps:
        return AlreadyThere()

    scale = safety_distance / d
    goal_x = person_x - dx * scale
    goal_y = person_y - dy * scale
    yaw = math.atan2(dy, dx)
    return GoalPose(x=goal_x, y=goal_y, yaw=yaw, frame_id=frame_id)


class EndpointSmoother:
    def __init__(self, alpha: float, jump_max: float):
        self.alpha = float(alpha)
        self.jump_max = float(jump_max)
        self._state: Optional[tuple[float, float]] = None

    def reset(self) -> None:
        self._state = None

    def update(self, x: float, y: float) -> tuple[float, float]:
        if self._state is None:
            self._state = (x, y)
            return self._state

        sx, sy = self._state
        if math.hypot(x - sx, y - sy) > self.jump_max:
            return self._state

        nx = self.alpha * x + (1.0 - self.alpha) * sx
        ny = self.alpha * y + (1.0 - self.alpha) * sy
        self._state = (nx, ny)
        return self._state


class ScanLatch:
    def __init__(self, maxlen: int = 20):
        self._buf: deque[tuple[float, object]] = deque(maxlen=maxlen)

    def add(self, scan, stamp_sec: float) -> None:
        self._buf.append((stamp_sec, scan))

    def get_scans_around(self, t_sec: float, dt_sec: float, max_scans: int = 3) -> list[object]:
        if not self._buf:
            return []
        candidates = [item for item in self._buf if abs(item[0] - t_sec) <= dt_sec]
        if not candidates:
            return []
        candidates.sort(key=lambda item: abs(item[0] - t_sec))
        return [scan for _, scan in candidates[:max_scans]]


class Phase1EndpointEngine:
    def __init__(self, zone: ServiceZone, params: Phase1Params):
        self.zone = zone
        self.params = params
        self.smoother = EndpointSmoother(params.smoother_alpha, params.jump_max)
        self.cluster_filter = ClusterFilterParams(
            n_min=params.n_min,
            width_min=params.width_min,
            width_max=params.width_max,
            range_min=params.range_min,
            range_max=params.range_max,
        )

    def reset(self) -> None:
        self.smoother.reset()

    def compute(
        self,
        trigger: TriggerEvent,
        scans: Iterable[object],
        robot_pose_xy: tuple[float, float],
        transform_laser_to_map: Callable[[float, float], tuple[float, float]] | None,
    ) -> tuple[PersonEndpoint | None, str]:
        if not trigger.hand_raised:
            return None, "trigger_off"
        if not trigger.motion_ok:
            return None, "motion_blocked"

        merged_pts = []
        for scan in scans:
            pts_laser = laser_scan_to_points(scan, frame_id="laser", max_range=self.params.max_scan_range)
            pts_map = transform_points_to_map(pts_laser, transform_laser_to_map, out_frame_id=self.zone.frame_id)
            merged_pts.extend(pts_map)

        if not merged_pts:
            return None, "no_points"

        pts_zone = filter_points_in_zone(merged_pts, self.zone)
        if not pts_zone:
            return None, "outside_zone"

        clusters = cluster_points(pts_zone, eps=self.params.cluster_eps)
        human_clusters = filter_human_clusters(clusters, self.cluster_filter)
        selected, status = select_phase1_cluster(human_clusters)
        if selected is None:
            return None, status

        sx, sy = self.smoother.update(selected.centroid_x, selected.centroid_y)
        return (
            PersonEndpoint(
                x=sx,
                y=sy,
                frame_id=self.zone.frame_id,
                stamp_sec=trigger.stamp_sec,
                source="lidar_native",
            ),
            "ok",
        )
