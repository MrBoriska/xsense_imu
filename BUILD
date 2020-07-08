load("//engine/build:isaac.bzl", "isaac_pkg", "isaac_py_app")

py_binary(
    name = "xsense_imu",
    visibility = ["//visibility:public"],
    srcs = [
        "__init__.py",
        "xsense_imu.py",
        "mtdef.py",
        "mtdevice.py"
    ],
    data = [
        "xsense_imu.app.json",
        "//packages:py_init",
        "requirements.txt"
    ],
    deps = [
        "//engine/pyalice"
    ],
)

isaac_pkg(
    name = "xsense_imu-pkg",
    srcs = ["xsense_imu"]
)

isaac_py_app(
    name = "svo_realsense_xsens_imu",
    srcs = [
        "svo_realsense_xsens_imu.py"
    ],
    data = [
        "svo_realsense_xsens_imu.config.json",
        "//packages:py_init",
    ],
    main = "svo_realsense_xsens_imu.py",
    modules = [
        "perception:stereo_visual_odometry",
        "realsense",
        "utils",
        "viewers",
    ],
    deps = [
        "//engine/pyalice",
        "//packages/xsense_imu:xsense_imu"
    ],
)

