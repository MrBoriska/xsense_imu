load("//engine/build:isaac.bzl", "isaac_app", "isaac_cc_library", "isaac_cc_module")

isaac_app(
     name = "xsense_imu",
     modules = [
       "//packages/xsense_imu:xsense_imu_components"
     ]
)

isaac_cc_module(
  name = "xsense_imu_components",
  copts = [
    "-Ipackages/xsense_imu/xspublic"
  ],
  linkopts = [
    "-Lpackages/xsense_imu/xspublic/xscommon",
    "-Lpackages/xsense_imu/xspublic/xscontroller",
    "-Lpackages/xsense_imu/xspublic/xstypes",
    "-lpthread",
    "-lrt",
    "-ldl"
  ],
  srcs = ["XSenseIMU.cpp"],
  hdrs = ["XSenseIMU.hpp", "XSenseCallbackHandler.hpp",],
  deps = [
    "//packages/xsense_imu/xspublic:xspublic"
  ]
)