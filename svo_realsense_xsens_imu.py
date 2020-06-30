'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
from engine.pyalice import Application, Node

import xsense_imu

if __name__ == '__main__':
    app = Application(app_filename="packages/xsense_imu/svo_realsense_xsens_imu.config.json")

    camera = app.add('camera').add(app.registry.isaac.RealsenseCamera)
    camera.config.align_to_color = False
    camera.config.auto_exposure_priority = False
    camera.config.enable_color = False
    camera.config.cols = 640
    camera.config.rows = 360
    camera.config.enable_depth = False
    camera.config.enable_depth_laser = False
    camera.config.enable_ir_stereo = True
    camera.config.ir_framerate = 30

    splitter_left = app.add('camera_splitter_left').add(
        app.registry.isaac.utils.ColorCameraProtoSplitter)
    splitter_left.config.only_pinhole = False

    splitter_right = app.add('camera_splitter_right').add(
        app.registry.isaac.utils.ColorCameraProtoSplitter)
    splitter_right.config.only_pinhole = False

    tracker = app.add('tracker').add(app.registry.isaac.StereoVisualOdometry)
    tracker.config.horizontal_stereo_camera = True
    tracker.config.process_imu_readings = True
    tracker.config.lhs_camera_frame = "left_ir_camera"
    tracker.config.rhs_camera_frame = "right_ir_camera"

    XSenseDriver = app.nodes["xsense_imu_node"].add(xsense_imu.XSensDriver)

    tracker.config.imu_frame = XSenseDriver.config.frame_id

    app.connect(XSenseDriver, "imu_raw", tracker, "imu")

    viewer = app.add('viewers').add(app.registry.isaac.viewers.ColorCameraViewer)
    viewer.config.reduce_scale = 2

    app.connect(camera, "left_ir", splitter_left, "color_camera")
    app.connect(splitter_left, "image", tracker, "left_image")
    app.connect(splitter_left, "intrinsics", tracker, "left_intrinsics")

    app.connect(camera, "right_ir", splitter_right, "color_camera")
    app.connect(splitter_right, "image", tracker, "right_image")
    app.connect(splitter_right, "intrinsics", tracker, "right_intrinsics")

    app.connect(camera, "left_ir", viewer, "color_listener")

    app.run()
