load("//engine/build:isaac.bzl", "isaac_app", "isaac_cc_library", "isaac_cc_module")

isaac_cc_library(
    name = "xspublic",
    visibility = ["//visibility:public"],
    srcs = glob([
      "xscommon/*.cpp",
      "xscontroller/*.cpp",
      "xstypes/*.cpp"
    ]),
    hdrs = glob([
      "xscontroller/*.h",
      "xscommon/*.h",
      "xstypes/*.h"
    ]),
    copts = [
      "-Ipackages/xsense_imu/xspublic",
      "-DHAVE_JOURNALLER",
      "-DHAVE_LIBUSB",
      "-Wno-unused-function"
    ],
    linkopts = [
      "-Lpackages/xsense_imu/xspublic/xscommon",
      "-Lpackages/xsense_imu/xspublic/xscontroller",
      "-Lpackages/xsense_imu/xspublic/xstypes",
      "-lpthread",
      "-lrt",
      "-ldl"
    ]
)