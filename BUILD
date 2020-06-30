load("//engine/build:isaac.bzl", "isaac_pkg")

py_binary(
    name = "xsense_imu",
    visibility = ["//visibility:public"],
    srcs = [
        "__init__.py",
        "xsense_imu.py",
    ],
    data = [
        "xsense_imu.app.json",
        "//packages:py_init"
    ],
    deps = [
        "//engine/pyalice"
    ],
)

isaac_pkg(
    name = "xsense_imu-pkg",
    srcs = ["xsense_imu"]
)