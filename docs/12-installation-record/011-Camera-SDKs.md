# SDK Installation
## RealSense ROS2 SDK

### Step 1: Install latest Intel® RealSense™ SDK 2.0

<u>Installation steps:</u>

1. Register the server's public key:

    ```sh
    sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
    ```

2. Add the server to the list of repositories:

    ```sh
    sudo add-apt-repository "deb https://librealsense.realsenseai.com/Debian/apt-repo $(lsb_release -cs) main" -u
    ```

3. Install the SDK:

    ```sh
    sudo apt-get install librealsense2-utils
    sudo apt-get install librealsense2-dev
    ```

4. Reconnect the RealSense device and run the following to verify the installation: `realsense-viewer`

### Step 2: Install ROS Wrapper for Intel® RealSense™ cameras

Install all realsense ROS packages by `sudo apt install ros-humble-realsense2-*`

## Usage

### Start the camera node

  #### with ros2 run:
    ros2 run realsense2_camera realsense2_camera_node
    # or, with parameters, for example - temporal and spatial filters are enabled:
    ros2 run realsense2_camera realsense2_camera_node --ros-args -p enable_color:=false -p spatial_filter.enable:=true -p temporal_filter.enable:=true

  #### with ros2 launch:
    ros2 launch realsense2_camera rs_launch.py
    ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 pointcloud.enable:=true
    ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true enable_sync:=true

You can see further usage instructions at [the official docs](https://github.com/realsenseai/realsense-ros?tab=readme-ov-file#usage).
<hr>
