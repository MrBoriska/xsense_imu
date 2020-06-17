load("//engine/build:isaac.bzl", "isaac_app", "isaac_cc_module")

isaac_app(
     name = "xsense_imu",
     modules = ["//packages/xsense_imu:xsense_imu_components"]
)

cc_library(
    name = "xsens_driver",
    visibility = ["//visibility:public"],
    linkopts = [
        "-Lxspublic/xscontroller",
        "-Lxspublic/xscommon",
        "-Lxspublic/xstypes",
        "-lxscontroller",
        "-lxscommon",
        "-lxstypes",
        "-lpthread",
        "-lrt",
        "-ldl"
    ],
    srcs = glob(["xspublic/*.cc"]),
    hdrs = glob(["xspublic/*.h"]),
)

isaac_cc_module(
  name = "xsense_imu_components",
  copts = [
    "-Ixspublic"
  ],
  srcs = ["XSenseIMU.cpp", "XSenseCallbackHandler.hpp"],
  hdrs = ["XSenseIMU.hpp"],
  deps = [
    ":xsens_driver"
  ],
)