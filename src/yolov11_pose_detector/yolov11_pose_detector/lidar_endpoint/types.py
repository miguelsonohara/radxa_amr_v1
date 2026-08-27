from dataclasses import dataclass, field
from typing import Optional


@dataclass(frozen=True)
class ServiceZone:
    frame_id: str
    x_min: float
    y_min: float
    x_max: float
    y_max: float
    epsilon: float = 0.0

    def contains(self, x: float, y: float) -> bool:
        return (
            (self.x_min - self.epsilon) <= x <= (self.x_max + self.epsilon)
            and (self.y_min - self.epsilon) <= y <= (self.y_max + self.epsilon)
        )


@dataclass(frozen=True)
class LidarPoint2D:
    x: float
    y: float
    frame_id: str
    theta: Optional[float] = None
    r: Optional[float] = None
    stamp_sec: Optional[float] = None


@dataclass
class LidarCluster:
    points: list[LidarPoint2D]
    centroid_x: float
    centroid_y: float
    frame_id: str
    n_points: int
    width: float
    mean_range: float


@dataclass(frozen=True)
class TriggerEvent:
    hand_raised: bool
    motion_ok: bool
    stamp_sec: float
    image_side_hint: Optional[str] = None


@dataclass(frozen=True)
class PersonEndpoint:
    x: float
    y: float
    frame_id: str
    stamp_sec: float
    source: str = "lidar_native"


@dataclass(frozen=True)
class GoalPose:
    x: float
    y: float
    yaw: float
    frame_id: str


@dataclass(frozen=True)
class AlreadyThere:
    reason: str = "already_there"


@dataclass
class Phase1Params:
    cluster_eps: float = 0.35
    n_min: int = 2
    width_min: float = 0.15
    width_max: float = 0.9
    range_min: float = 0.4
    range_max: float = 8.0
    safety_distance: float = 1.0
    safety_epsilon: float = 0.1
    smoother_alpha: float = 0.45
    jump_max: float = 1.0
    scan_latch_size: int = 20
    latch_dt_sec: float = 0.15
    scans_to_merge: int = 3
    transform_timeout_sec: float = 0.05
    max_scan_range: float = 8.0
    debug: bool = False
    extra: dict = field(default_factory=dict)


@dataclass
class Phase2Params:
    v_stand_max: float = 0.3
    assoc_gate: float = 0.9
    age_min: int = 3
    miss_max: int = 8
    side_angle_alpha_deg: float = 18.0
    t_lost_sec: float = 2.0
    dt_sync_sec: float = 0.2
    goal_update_policy: str = "latch"


@dataclass
class PersonTrack:
    track_id: int
    x: float
    y: float
    vx: float
    vy: float
    stamp_sec: float
    age_frames: int = 1
    hits: int = 1
    misses: int = 0
    in_zone: bool = True
    standing: bool = True
    side_lidar: str = "center"

    @property
    def speed(self) -> float:
        return float((self.vx ** 2 + self.vy ** 2) ** 0.5)


@dataclass(frozen=True)
class ImageSideHint:
    side: str
    area_rank: int = 0


@dataclass(frozen=True)
class AssociationResult:
    ok: bool
    method: str
    track_id: Optional[int] = None
    reason: str = ""


@dataclass
class LockedTarget:
    track_id: int
    lock_stamp_sec: float
    last_seen_sec: float
    active: bool = True


@dataclass(frozen=True)
class WaveEvent:
    track_id: int
    t_wave: float
    x: float
    y: float
