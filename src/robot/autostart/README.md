# Robot autostart button

This single service:

1. loads the target user's Bash environment and ROS workspaces;
2. starts and supervises the micro-ROS UDP agent;
3. plays the ESP32 ready sound once it connects;
4. starts `robot/planner.launch.py` when physical header pin 29 is pulled low.

It does not hard-code `ROS_DOMAIN_ID` or `XRCE_DOMAIN_ID_OVERRIDE`. Set them
in the user's Bash profile; `robot/scripts/setup_ros_env.sh` supplies its
existing defaults only when they are unset.

Optional exported settings:

```bash
export WRO_BUTTON_PIN=29
export WRO_DRIVE_MOTOR=100
export WRO_DEBUG_VIEW=false
export MICRO_ROS_PORT=8888
```

Install from the Jetson:

```bash
cd ~/robot_ws/robot/autostart
sudo ./install.sh
journalctl -u wro-robot-autostart.service -f
```

If Jetson GPIO is unavailable, the installer installs the
`python3-jetson-gpio` apt package automatically.

Remove it with:

```bash
sudo ./uninstall.sh
```
