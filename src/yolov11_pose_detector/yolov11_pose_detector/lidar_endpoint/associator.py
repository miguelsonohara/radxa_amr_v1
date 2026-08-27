from __future__ import annotations

import math

from .types import AssociationResult, ImageSideHint, LockedTarget, PersonTrack, Phase2Params, TriggerEvent


def classify_track_side(
    track: PersonTrack,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    alpha_deg: float,
) -> str:
    dx = track.x - robot_x
    dy = track.y - robot_y
    bearing = math.atan2(dy, dx) - robot_yaw
    bearing = math.atan2(math.sin(bearing), math.cos(bearing))
    alpha = math.radians(alpha_deg)
    if bearing > alpha:
        return "left"
    if bearing < -alpha:
        return "right"
    return "center"


class TriggerAssociator:
    def __init__(self, params: Phase2Params):
        self.params = params

    def associate(
        self,
        trigger: TriggerEvent,
        tracks: list[PersonTrack],
        side_hint: ImageSideHint | None = None,
    ) -> AssociationResult:
        candidates = [
            t
            for t in tracks
            if t.standing and t.in_zone and t.age_frames >= self.params.age_min and abs(t.stamp_sec - trigger.stamp_sec) <= self.params.dt_sync_sec
        ]
        if len(candidates) == 0:
            return AssociationResult(ok=False, method="reject", reason="no_standing_candidate")
        if len(candidates) == 1:
            return AssociationResult(ok=True, method="unique_standing", track_id=candidates[0].track_id)

        if side_hint is not None:
            side_filtered = [t for t in candidates if t.side_lidar == side_hint.side]
            if len(side_filtered) == 1:
                return AssociationResult(ok=True, method="side_hint", track_id=side_filtered[0].track_id)
            if len(side_filtered) > 1:
                candidates = side_filtered

        candidates.sort(key=lambda t: math.hypot(t.x, t.y))
        if len(candidates) == 1:
            return AssociationResult(ok=True, method="nearest", track_id=candidates[0].track_id)
        return AssociationResult(ok=False, method="reject", reason="ambiguous_multi_standing")


def update_lock_state(
    lock: LockedTarget | None,
    assoc: AssociationResult | None,
    now_sec: float,
    t_lost_sec: float,
    track_exists: bool,
) -> LockedTarget | None:
    if assoc is not None and assoc.ok and assoc.track_id is not None:
        return LockedTarget(track_id=assoc.track_id, lock_stamp_sec=now_sec, last_seen_sec=now_sec, active=True)
    if lock is None:
        return None
    if track_exists:
        lock.last_seen_sec = now_sec
        return lock
    if now_sec - lock.last_seen_sec > t_lost_sec:
        return None
    return lock
