import pytest
import numpy as np
from yolov11_pose_detector.pose_detector_node import YoloV11PoseDetectorNode


class MockTFBuffer:
    def can_transform(self, *args):
        return False


class MockPoseDetectorNode(YoloV11PoseDetectorNode):
    """Subclass of YoloV11PoseDetectorNode overriding ROS 2 init for pure unit testing."""
    def __init__(self):
        self.c_x = 320.0
        self.c_y = 240.0
        self.f_x = 550.0
        self.f_y = 550.0
        self.kp_conf_threshold = 0.5
        self.person_shoulder_width = 0.38
        self.safety_distance = 1.4
        self.camera_laser_yaw_offset = 0.0
        self.camera_yaw_offset_rad = 0.0
        self.camera_frame_id = 'camera_color_optical_frame'
        self.laser_frame_id = 'laser'
        self.lidar_gate_angle_rad = np.radians(5.0)
        self.arm_raise_head_margin = 0.15
        self.arm_raise_elbow_margin = 0.20
        self.tf_buffer = MockTFBuffer()
        self.latest_scan = None


@pytest.fixture
def node():
    return MockPoseDetectorNode()


def test_is_arm_raised_left_arm_high_raise(node):
    """Verifies left arm raised detection when wrist is significantly above head top, elbow, and shoulder."""
    kpts = np.zeros((17, 2), dtype=np.float32)
    kpts_conf = np.ones(17, dtype=np.float32)

    # Nose (0): y=100. Shoulders (5, 6): dx=200 -> shoulder_dist=200. Head top y=100-30=70.
    kpts[0] = [300.0, 100.0]
    kpts[5] = [200.0, 200.0]  # L_Shoulder
    kpts[6] = [400.0, 200.0]  # R_Shoulder

    # L_Elbow (7): y=80, L_Wrist (9): y=20 (high up in air, x=150 away from head center)
    kpts[7] = [180.0, 80.0]
    kpts[9] = [150.0, 20.0]

    # Right arm down
    kpts[8] = [420.0, 250.0]
    kpts[10] = [420.0, 300.0]

    assert node._is_arm_raised(kpts, kpts_conf) is True


def test_is_arm_raised_head_clutch_rejected(node):
    """Verifies False return when person holds head with 2 hands ('ôm đầu')."""
    kpts = np.zeros((17, 2), dtype=np.float32)
    kpts_conf = np.ones(17, dtype=np.float32)

    # Shoulders at y=200, Nose at y=100 -> head_center_x=300, shoulder_dist=200
    kpts[0] = [300.0, 100.0]
    kpts[5] = [200.0, 200.0]
    kpts[6] = [400.0, 200.0]

    # Both wrists touching/clutching near head level (y=90, x near 300)
    kpts[7] = [220.0, 120.0]
    kpts[9] = [280.0, 90.0]  # L_Wrist near head

    kpts[8] = [380.0, 120.0]
    kpts[10] = [320.0, 90.0]  # R_Wrist near head

    assert node._is_arm_raised(kpts, kpts_conf) is False


def test_is_arm_raised_slightly_above_head_rejected(node):
    """Verifies False return when hand is only slightly above head or resting near head level."""
    kpts = np.zeros((17, 2), dtype=np.float32)
    kpts_conf = np.ones(17, dtype=np.float32)

    kpts[0] = [300.0, 100.0]
    kpts[5] = [200.0, 200.0]  # shoulder_dist = 200, head top y ~ 70
    kpts[6] = [400.0, 200.0]

    # L_Wrist at y=65 (only slightly above head top y=70, margin threshold requires y < 70 - 30 = 40)
    kpts[7] = [180.0, 110.0]
    kpts[9] = [180.0, 65.0]

    kpts[8] = [420.0, 250.0]
    kpts[10] = [420.0, 300.0]

    assert node._is_arm_raised(kpts, kpts_conf) is False


def test_is_arm_raised_both_down(node):
    """Verifies return False when both arms are below shoulders."""
    kpts = np.zeros((17, 2), dtype=np.float32)
    kpts_conf = np.ones(17, dtype=np.float32)

    kpts[0] = [300.0, 100.0]
    kpts[5] = [200.0, 200.0]
    kpts[7] = [200.0, 250.0]
    kpts[9] = [200.0, 300.0]
    kpts[6] = [400.0, 200.0]
    kpts[8] = [400.0, 250.0]
    kpts[10] = [400.0, 300.0]

    assert node._is_arm_raised(kpts, kpts_conf) is False


def test_select_target_person_multi_person(node):
    """Verifies Smart Multi-Person Selection picks the candidate waving arm."""
    kpts_waving = np.zeros((17, 2), dtype=np.float32)
    kpts_waving[0] = [310.0, 100.0]
    kpts_waving[5] = [210.0, 200.0]
    kpts_waving[6] = [410.0, 200.0]
    kpts_waving[7] = [180.0, 80.0]
    kpts_waving[9] = [150.0, 20.0]

    kpts_bystander = np.zeros((17, 2), dtype=np.float32)
    kpts_bystander[0] = [100.0, 100.0]
    kpts_bystander[5] = [50.0, 200.0]
    kpts_bystander[6] = [150.0, 200.0]
    kpts_bystander[7] = [100.0, 250.0]
    kpts_bystander[9] = [100.0, 300.0]

    detections = [
        {
            'box': np.array([50.0, 100.0, 150.0, 400.0]),
            'score': 0.9,
            'kpts': kpts_bystander,
            'kpts_conf': np.ones(17, dtype=np.float32)
        },
        {
            'box': np.array([260.0, 100.0, 360.0, 400.0]),
            'score': 0.95,
            'kpts': kpts_waving,
            'kpts_conf': np.ones(17, dtype=np.float32)
        }
    ]

    target_idx, target_det, is_waving = node._select_target_person(detections)

    assert target_idx == 1
    assert is_waving is True
    assert target_det['score'] == 0.95


def test_process_person_tracking_low_pixel_width(node):
    """Verifies that shoulder pixel width <= 5.0 returns None to prevent zero division."""
    kpts = np.zeros((17, 2), dtype=np.float32)
    kpts[5] = [200.0, 200.0]
    kpts[6] = [202.0, 200.0]  # dx = 2 pixels (< 5.0)

    result = node._process_person_tracking(kpts, np.ones(17, dtype=np.float32))
    assert result is None


def test_compute_body_centroid_robust_against_arm_raise(node):
    """Verifies that when one shoulder/arm is pulled sideways, head and hip anchor preserve the true body centerline."""
    kpts = np.zeros((17, 2), dtype=np.float32)
    kpts_conf = np.ones(17, dtype=np.float32)

    # True body centerline at x=320
    kpts[0] = [320.0, 100.0]  # Nose at 320
    kpts[1] = [310.0, 95.0]   # L Eye
    kpts[2] = [330.0, 95.0]   # R Eye

    # Normal hips centered at x=320
    kpts[11] = [280.0, 350.0]  # L Hip
    kpts[12] = [360.0, 350.0]  # R Hip -> hip midpoint = 320

    # Shoulder asymmetric (e.g. left shoulder pulled to x=220, right shoulder at x=400 -> midpoint = 310)
    kpts[5] = [220.0, 190.0]
    kpts[6] = [400.0, 200.0]

    u, v = node._compute_body_centroid(kpts, kpts_conf)
    # The median of head (320), hips (320), and shoulders (310) is exactly 320.0!
    assert abs(u - 320.0) < 1.0


class MockLaserScan:
    def __init__(self, ranges, angle_min=-np.pi, angle_max=np.pi):
        self.ranges = ranges
        self.range_min = 0.25
        self.range_max = 12.0
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.angle_increment = (angle_max - angle_min) / len(ranges)


def test_fuse_lidar_distance_rejects_background_wall(node):
    """
    Verifies that when a person is at 1.8m and a background wall is at 3.2m within the scan cone,
    foreground clustering isolates the person at ~1.8m instead of returning the wall distance.
    """
    # 360 ranges from -pi to +pi (0 is forward, index 180 is theta=0)
    num_points = 360
    ranges = [5.0] * num_points

    # Place person at theta ≈ 0 (indices 178-182) at 1.85m
    # and place background wall at adjacent beam index (183-185) at 3.20m
    ranges[178] = 1.80
    ranges[179] = 1.82
    ranges[180] = 1.85
    ranges[181] = 1.88
    ranges[182] = 1.89
    ranges[183] = 3.20
    ranges[184] = 3.25
    ranges[185] = 3.30

    node.latest_scan = MockLaserScan(ranges)

    # Person body center at center of camera (u=320, v=240), estimated distance Z_est=2.0m
    best_x, best_y, fused_dist = node._fuse_lidar_distance(320.0, 240.0, Z_est=2.0)

    # Must pick the foreground person (~1.85m), NOT the background wall (~3.25m)
    assert best_x is not None
    assert 1.75 <= fused_dist <= 1.95


def test_fuse_lidar_distance_wide_angle_tolerance(node):
    """
    Verifies that when a person is at +10 degrees off-center (u=420),
    the wide-angle search sector (+/- 20 deg) successfully finds and tracks the person.
    """
    num_points = 360
    ranges = [5.0] * num_points

    # +10 degrees is at index 180 + 10 = 190
    for idx in range(188, 193):
        ranges[idx] = 2.20

    node.latest_scan = MockLaserScan(ranges)

    # Sight ray pointing at ~ +10 deg
    best_x, best_y, fused_dist = node._fuse_lidar_distance(420.0, 240.0, Z_est=2.2)

    assert best_x is not None
    assert 2.10 <= fused_dist <= 2.30




