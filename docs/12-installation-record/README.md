# Detailed Installation Record

This directory preserves the full step-by-step installation notes, screenshots, command history, setup alternatives, and troubleshooting observations from the vehicle build. It is intentionally separate from Chapter 12: [the chapter](../12-build-and-operation-guide.md) provides the current reconstruction and operating route, while these documents retain the detail that can be revised without compressing the historical record.

Personal user names, host names, local absolute paths, and recorded network credentials have been replaced with neutral placeholders such as `$HOME`, `<team-user>`, `<jetson-host>`, `<WIFI_SSID>`, and `<WIFI_PASSWORD>`. Replace placeholders only with the current team deployment values.

## Jetson and ROS 2

- [Jetson initial setup](001-jetson.md)
- [Install ROS 2](002-ros2.md)
- [Install base dependencies](003-base-dependency.md)
- [ESP32-S3 USB detection](004-esp32s3-usb-detect.md)
- [Ethernet, kernel modules, and user groups](008-add-module-config-eth-user-group.md)
- [VNC setup record](010-VNC.md)

## ESP32, Arduino, and micro-ROS

- [Arduino IDE and ESP32 setup](005-Arduino-01.md)
- [Arduino on Jetson](005-Arduino-Jetson.md)
- [Arduino on macOS](005-Arduino-MacBook.md)
- [micro-ROS Agent on Jetson](006-microROSAgent-Jetson.md)
- [SSH-Agent Git workflow record](007-jetson-use-mac-ssh-agent-to-push-github.md)

## Sensors, calibration, and application experiments

- [LiDAR installation and validation](009-LiDAR.md)
- [Camera SDK installation](011-Camera-SDKs.md)
- [Calibration attempt record](012-calib-no-use.md)
- [YOLO experiment record](013-YOLO-AI.md)
- [TM171 IMU installation](014-imu.md)

## Screenshot assets

All screenshots referenced by these records are stored in [images](images/). They preserve the original step context; Chapter 12 itself remains the concise, current operating guide.
