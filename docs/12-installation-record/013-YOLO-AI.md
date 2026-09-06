
# Jetson-Inference

### Build from source
[Instructions](https://github.com/dusty-nv/jetson-inference/blob/master/docs/building-repo-2.md#building-the-project-from-source)

```
sudo apt-get update
sudo apt-get install git cmake libpython3-dev python3-numpy
git clone --recursive --depth=1 https://github.com/dusty-nv/jetson-inference
cd jetson-inference
mkdir build
cd build
cmake ../
```

I got an error at the `make` command.

Edit `jetson-inference/utils/python/bindings/CMakeLists.txt` and comment out:
```
if(${NUMPY_FOUND})
        target_link_libraries(jetson-utils-python-${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR} npymath)
endif()
```

to


```
if(${NUMPY_FOUND})
       # target_link_libraries(jetson-utils-python-${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR} npymath)
endif()
```

Then, continue:

```
make
sudo make install
sudo ldconfig
```

I got a couple errors, so I decided to try to install YOLO.

<hr>

# YOLO using ROS2

[Instructions](https://github.com/mgonzs13/yolo_ros?tab=readme-ov-file#usage)
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/mgonzs13/yolo_ros.git
pip3 install -r yolo_ros/requirements.txt
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

# Clone this repo
```
mkdir -p ~/yolo_ws/src
cd ~/yolo_ws/src
git clone https://github.com/mgonzs13/yolo_ros.git
```


# Install uv and python dependencies
```
curl -LsSf https://astral.sh/uv/install.sh | sh
source ~/.bashrc
cd ~/yolo_ws/src/yolo_ros
uv sync
```

# Install rosdep dependencies and build
```
cd ~/yolo_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

```
export ROS_DOMAIN_ID=30
export XRCE_DOMAIN_ID_OVERRIDE=30
ros2 launch yolo_bringup yolo.launch.py \
  model:=$HOME/wro_ign_gazebo_sim/install/hiway_perception/share/hiway_perception/model/wro_obstacle_detector.pt \
  input_image_topic:=/camera/camera/color/image_raw \
  input_depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  input_depth_info_topic:=/camera/camera/color/camera_info \
  use_3d:=True \
  target_frame:=camera_color_optical_frame \
  device:=cpu
```

```
ros2 topic echo /yolo/detections_3d
```



#No need
Then,

```
docker build -t yolo_ros .
docker run -it --rm --gpus all yolo_ros
```

Inside docker we can run:

```
export ROS_DOMAIN_ID=30
export XRCE_DOMAIN_ID_OVERRIDE=30
ros2 launch yolo_bringup yolov5.launch.py input_image_topic:=/camera/camera/color/image_raw
```

We will get an error that an invalid CUDA device was requested. Install the [NVIDIA container toolkit](#nvidia-container-toolkit), then proceed.

I can't seem to get GPU acceleration working for now, so use:

```
export ROS_DOMAIN_ID=30
export XRCE_DOMAIN_ID_OVERRIDE=30
ros2 launch yolo_bringup yolov5.launch.py input_image_topic:=/camera/camera/color/image_raw device:=cpu
```

<hr>

# NVIDIA Container Toolkit
[Reference](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#with-apt-ubuntu-debian)

```
sudo apt-get update && sudo apt-get install -y --no-install-recommends \
   curl \
   gnupg2

curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update

export NVIDIA_CONTAINER_TOOLKIT_VERSION=1.18.1-1
  sudo apt-get install -y \
      nvidia-container-toolkit=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      nvidia-container-toolkit-base=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      libnvidia-container-tools=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      libnvidia-container1=${NVIDIA_CONTAINER_TOOLKIT_VERSION}
```

Then, configure Docker:

```
sudo nvidia-ctk runtime configure --runtime=docker

sudo systemctl restart docker
```

<hr>

# YOLO with RealSense Camera

In an outside terminal:
```
export ROS_DOMAIN_ID=30
export XRCE_DOMAIN_ID_OVERRIDE=30
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=True
```

In another terminal:
```
docker run -it --rm --gpus all yolo_ros
```

In Docker:
```
export ROS_DOMAIN_ID=30
export XRCE_DOMAIN_ID_OVERRIDE=30
ros2 launch yolo_bringup yolo.launch.py input_image_topic:=/camera/camera/color/image_raw input_depth_topic:=/camera/camera/depth/image_rect_raw input_depth_info_topic:=/camera/camera/depth/camera_info device:=cpu use_3d:=True target_frame:=camera_link
```

Open RViz2.

Under **Global Options**, set `Fixed Frame` to `camera_link`.

At the bottom left,

Click **Add**, then click `By Topic` at the top, and click `/camera` -> `/camera` -> `/aligned_depth_to_color` -> `/image_raw` -> `DepthCloud`.

Click **Ok**.

Now, in the left panel, **Displays**, you will see a new item, **DepthCloud**.

Expand **DepthCloud**, and set **Depth Map Topic** to `/camera/camera/aligned_depth_to_color/image_raw`.

Set **Color Image Topic** to `/camera/camera/color/image_raw`.

At the right panel, **Views**, change **Type** to `FrameAligned`.

Change **Point towards** to `+x axis`, then click `Zero` next to **Type**.
