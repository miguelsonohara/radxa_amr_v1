# ---------------------------------------------------------------------
# Copyright (c) 2025 Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause
# ---------------------------------------------------------------------

from qai_hub_models.models.yolov11_pose.app import YoloV11PoseApp as App

from .model import MODEL_ID
from .model import YoloV11PoseDetector as Model

__all__ = ["MODEL_ID", "App", "Model"]
