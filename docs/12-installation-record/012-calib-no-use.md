```
pip install ros2-calib "numpy<1.25"
ros2_calib
```

You might need

```
sudo apt install libxcb-cursor0
```

Record rosbag (.mcap)
```
sudo apt update
sudo apt install ros-humble-rosbag2-storage-mcap
ros2 bag record -s mcap -a
```
