load("//engine/build:isaac.bzl", "isaac_app", "isaac_cc_library", "isaac_cc_module")


isaac_cc_library(
    name = "xspublic",
    visibility = ["//visibility:public"],
    srcs = glob([
      "xscommon/*.o",
      "xscontroller/*.o",
      "xstypes/*.o",
    ]),
    hdrs = glob([
      "xscontroller/*.h",
      "xscommon/*.h",
      "xstypes/*.h"
    ]),
    copts = [
      "-Ipackages/xsense_imu/xspublic",
    ]
)