from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .cluster import ClusterFilterParams, cluster_points, filter_human_clusters
from .types import LidarPoint2D, PersonTrack, Phase2Params


class KalmanCV2D:
    def __init__(self, x: float, y: float, q: float = 0.2, r: float = 0.15):
        self.x = np.array([[x], [y], [0.0], [0.0]], dtype=float)
        self.p = np.eye(4, dtype=float) * 1.0
        self.q = float(q)
        self.r = float(r)

    def predict(self, dt: float) -> tuple[float, float, float, float]:
        dt = max(float(dt), 1e-3)
        f = np.array(
            [[1.0, 0.0, dt, 0.0], [0.0, 1.0, 0.0, dt], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
            dtype=float,
        )
        dt2 = dt * dt
        q = self.q * np.array(
            [[dt2, 0.0, dt, 0.0], [0.0, dt2, 0.0, dt], [dt, 0.0, 1.0, 0.0], [0.0, dt, 0.0, 1.0]],
            dtype=float,
        )
        self.x = f @ self.x
        self.p = f @ self.p @ f.T + q
        return tuple(self.x.flatten())

    def update(self, mx: float, my: float) -> tuple[float, float, float, float]:
        z = np.array([[mx], [my]], dtype=float)
        h = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]], dtype=float)
        r = np.eye(2, dtype=float) * self.r
        y = z - (h @ self.x)
        s = h @ self.p @ h.T + r
        k = self.p @ h.T @ np.linalg.inv(s)
        self.x = self.x + k @ y
        self.p = (np.eye(4) - (k @ h)) @ self.p
        return tuple(self.x.flatten())


@dataclass
class _TrackState:
    kf: KalmanCV2D
    track: PersonTrack


def _associate_greedy(
    predicted: list[tuple[int, float, float]],
    measures: list[tuple[float, float]],
    gate: float,
) -> tuple[list[tuple[int, int]], list[int], list[int]]:
    pairs: list[tuple[float, int, int]] = []
    for t_idx, tx, ty in predicted:
        for m_idx, (mx, my) in enumerate(measures):
            d = math.hypot(tx - mx, ty - my)
            if d <= gate:
                pairs.append((d, t_idx, m_idx))
    pairs.sort(key=lambda item: item[0])

    used_t: set[int] = set()
    used_m: set[int] = set()
    matches: list[tuple[int, int]] = []
    for _, t_idx, m_idx in pairs:
        if t_idx in used_t or m_idx in used_m:
            continue
        used_t.add(t_idx)
        used_m.add(m_idx)
        matches.append((t_idx, m_idx))

    unmatched_t = [idx for idx, _, _ in predicted if idx not in used_t]
    unmatched_m = [idx for idx in range(len(measures)) if idx not in used_m]
    return matches, unmatched_t, unmatched_m


class LidarPersonTracker:
    def __init__(self, params: Phase2Params, cluster_filter: ClusterFilterParams, cluster_eps: float):
        self.params = params
        self.cluster_filter = cluster_filter
        self.cluster_eps = float(cluster_eps)
        self._next_id = 1
        self._tracks: dict[int, _TrackState] = {}

    def get_track(self, track_id: int) -> PersonTrack | None:
        st = self._tracks.get(track_id)
        return st.track if st else None

    def update_points(self, points_in_zone: list[LidarPoint2D], stamp_sec: float) -> list[PersonTrack]:
        clusters = cluster_points(points_in_zone, eps=self.cluster_eps)
        human_clusters = filter_human_clusters(clusters, self.cluster_filter)
        measurements = [(c.centroid_x, c.centroid_y) for c in human_clusters]
        return self.update_measurements(measurements, stamp_sec)

    def update_measurements(self, measurements: list[tuple[float, float]], stamp_sec: float) -> list[PersonTrack]:
        predicted: list[tuple[int, float, float]] = []
        for tid, state in self._tracks.items():
            dt = max(stamp_sec - state.track.stamp_sec, 1e-3)
            px, py, _, _ = state.kf.predict(dt)
            predicted.append((tid, px, py))

        matches, unmatched_t, unmatched_m = _associate_greedy(predicted, measurements, self.params.assoc_gate)

        for tid, m_idx in matches:
            state = self._tracks[tid]
            mx, my = measurements[m_idx]
            x, y, vx, vy = state.kf.update(mx, my)
            tr = state.track
            tr.x, tr.y, tr.vx, tr.vy = float(x), float(y), float(vx), float(vy)
            tr.stamp_sec = stamp_sec
            tr.age_frames += 1
            tr.hits += 1
            tr.misses = 0
            tr.standing = tr.speed < self.params.v_stand_max

        for tid in unmatched_t:
            tr = self._tracks[tid].track
            tr.misses += 1
            tr.age_frames += 1
            tr.stamp_sec = stamp_sec
            tr.standing = tr.speed < self.params.v_stand_max

        for m_idx in unmatched_m:
            mx, my = measurements[m_idx]
            tid = self._next_id
            self._next_id += 1
            tr = PersonTrack(track_id=tid, x=mx, y=my, vx=0.0, vy=0.0, stamp_sec=stamp_sec)
            self._tracks[tid] = _TrackState(kf=KalmanCV2D(mx, my), track=tr)

        dead_ids = [tid for tid, st in self._tracks.items() if st.track.misses > self.params.miss_max]
        for tid in dead_ids:
            del self._tracks[tid]

        return [st.track for st in self._tracks.values()]
