import pytest
import numpy as np
from yolov11_pose_detector.pose_detector_node import YoloV11PoseDetectorNode


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
        self.arm_raise_head_margin = 0.15
        self.arm_raise_elbow_margin = 0.20


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

