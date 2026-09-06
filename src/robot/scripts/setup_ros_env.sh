export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
export XRCE_DOMAIN_ID_OVERRIDE="${XRCE_DOMAIN_ID_OVERRIDE:-${ROS_DOMAIN_ID}}"
source /opt/ros/humble/setup.bash
if [ -d ~/ldlidar_ros2_ws/src/ldlidar_ros2 ]; then
    cd ~/ldlidar_ros2_ws && source install/setup.bash
elif [ -d ~/fork_ldlidar_ros2_ws/src/ldlidar_ros2 ]; then
    cd ~/fork_ldlidar_ros2_ws && source install/setup.bash
else
    echo "No lidar workspace found"
    exit 1
fi
cd ~/laser_merger_ws && source install/setup.bash
if [ -d ~/imu_ws ]; then
    cd ~/imu_ws && source install/setup.bash
elif [ -d ~/tm-imu ]; then
    cd ~/tm-imu && source install/setup.bash
else
    echo "No imu workspace found"
    exit 1
fi
cd ~/robot_ws && source install/setup.bash
cd ~/microros_ws && source install/setup.bash
cd ~/robot_ws
