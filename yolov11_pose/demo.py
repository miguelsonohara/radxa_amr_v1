# ---------------------------------------------------------------------
# Copyright (c) 2025 Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause
# ---------------------------------------------------------------------


from qai_hub_models.models._shared.yolo.demo import yolo_pose_estimation_demo
from qai_hub_models.models.yolov11_pose.app import YoloV11PoseApp
from qai_hub_models.models.yolov11_pose.model import (
    MODEL_ASSET_VERSION,
    MODEL_ID,
    YoloV11PoseDetector,
)
from qai_hub_models.utils.asset_loaders import CachedWebModelAsset

IMAGE_ADDRESS = CachedWebModelAsset.from_asset_store(
    MODEL_ID, MODEL_ASSET_VERSION, "yolov11_pose_demo.jpg"
)


def main(is_test: bool = False) -> None:
    yolo_pose_estimation_demo(
        model_type=YoloV11PoseDetector,
        model_id=MODEL_ID,
        app_type=YoloV11PoseApp,
        default_image=IMAGE_ADDRESS,
        output_filename="yolov11_pose_demo_output.png",
        is_test=is_test,
    )


if __name__ == "__main__":
    main()
