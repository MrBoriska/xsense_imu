# xsense_imu
xsense imu driver for Isaac SDK

Support only `ImuProto` message.


## Requirements
```
sudo apt install python-numba
sudo apt install python-scipy
pip3 install -r requirements.txt
```

For Jetson TX2 needs for available /dev/ttyUSB0

may be, this help for you: https://github.com/xsens/xsens_mti_ros_node#prerequisites

#### Or (Extended)

https://base.xsens.com/hc/en-us/articles/360014235960-Interfacing-MTi-devices-with-the-NVIDIA-Jetson

#### Or

for L4T kernel menuconfig (_good working for me_)

`CONFIG_USB_SERIAL_XSENS_MT=y`

(https://github.com/Darthussa/jetson_tx2_custom_board/commit/be295f2dad64deaec074c4fbd9b4da2fbdbc0a28#diff-d407671e9873ccc37a42070ac93e5ecdR933)

