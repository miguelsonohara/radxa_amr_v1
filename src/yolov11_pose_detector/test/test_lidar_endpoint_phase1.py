import math
from types import SimpleNamespace

from yolov11_pose_detector.lidar_endpoint.cluster import (
    ClusterFilterParams,
    cluster_points,
    filter_human_clusters,
    select_phase1_cluster,
)
from yolov11_pose_detector.lidar_endpoint.endpoint import (
    EndpointSmoother,
    Phase1EndpointEngine,
    ScanLatch,
    compute_safety_goal,
)
from yolov11_pose_detector.lidar_endpoint.types import LidarPoint2D, Phase1Params, ServiceZone, TriggerEvent
from yolov11_pose_detector.lidar_endpoint.zone import filter_points_in_zone


def test_zone_contains_boundary():
    zone = ServiceZone(frame_id="map", x_min=0.0, y_min=0.0, x_max=2.0, y_max=2.0, epsilon=0.0)
    assert zone.contains(0.0, 0.0)
    assert zone.contains(2.0, 2.0)
    assert not zone.contains(-0.01, 1.0)


def test_cluster_and_filter():
    pts = [
        LidarPoint2D(1.0, 0.0, "map"),
        LidarPoint2D(1.05, 0.0, "map"),
        LidarPoint2D(2.0, 2.0, "map"),
        LidarPoint2D(2.04, 2.02, "map"),
    ]
    clusters = cluster_points(pts, eps=0.15)
    assert len(clusters) == 2
    filtered = filter_human_clusters(
        clusters,
        ClusterFilterParams(n_min=2, width_min=0.01, width_max=0.5, range_min=0.5, range_max=5.0),
    )
    selected, status = select_phase1_cluster(filtered)
    assert status == "ambiguous"
    assert selected is None


def test_compute_safety_goal_distance():
    goal = compute_safety_goal(2.0, 0.0, 0.0, 0.0, safety_distance=1.0)
    assert abs(goal.x - 1.0) < 1e-6
    assert abs(goal.y) < 1e-6
    assert abs(goal.yaw) < 1e-6


def test_endpoint_smoother_jump_reject():
    sm = EndpointSmoother(alpha=0.5, jump_max=0.2)
    sm.update(1.0, 1.0)
    x, y = sm.update(5.0, 5.0)
    assert abs(x - 1.0) < 1e-6
    assert abs(y - 1.0) < 1e-6


def test_scan_latch_and_engine():
    scan = SimpleNamespace(
        ranges=[1.0, 1.02, float("inf")],
        angle_min=0.0,
        angle_increment=0.05,
        range_min=0.1,
        range_max=8.0,
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)),
    )
    latch = ScanLatch(maxlen=5)
    latch.add(scan, 1.0)
    scans = latch.get_scans_around(1.0, 0.05, 1)
    assert len(scans) == 1

    zone = ServiceZone(frame_id="map", x_min=-2, y_min=-2, x_max=2, y_max=2)
    params = Phase1Params(n_min=2, width_min=0.0, width_max=1.0, cluster_eps=0.2)
    engine = Phase1EndpointEngine(zone, params)
    trigger = TriggerEvent(hand_raised=True, motion_ok=True, stamp_sec=1.0)

    endpoint, status = engine.compute(
        trigger=trigger,
        scans=scans,
        robot_pose_xy=(0.0, 0.0),
        transform_laser_to_map=lambda x, y: (x, y),
    )
    assert status == "ok"
    assert endpoint is not None
    assert math.hypot(endpoint.x, endpoint.y) > 0.5


def test_filter_points_in_zone():
    zone = ServiceZone(frame_id="map", x_min=-1, y_min=-1, x_max=1, y_max=1)
    pts = [LidarPoint2D(0.0, 0.0, "map"), LidarPoint2D(2.0, 2.0, "map")]
    out = filter_points_in_zone(pts, zone)
    assert len(out) == 1
