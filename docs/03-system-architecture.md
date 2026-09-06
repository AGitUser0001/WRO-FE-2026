# System Architecture

## Purpose

Explain how the vehicle organizes its mechanical, power, sensing, computing, communication, software-control, and safety interfaces as one system. This chapter records cross-subsystem connections and responsibilities; the corresponding chapters cover mechanical detail, power budget, sensor calibration, and software algorithms.

## Summary

The vehicle uses one 12 V main battery. Competition rules allow only two operations: the first operates the main power switch to power the complete vehicle; the second operates the Jetson-GPIO program-start button. The button pulls physical Jetson pin `29` low, and the `robot_autostart` service starts `planner.launch.py`, causing the vehicle to begin running. The Jetson Orin Nano Super Developer Kit performs perception, state estimation, and decision-making, while the ESP32-S3 and carrier board perform actuator-side control. Two LDROBOT STL-27L 2D LiDARs, an Intel RealSense D435f depth camera, and a TM171 IMU provide environment and vehicle-state information. The GA25-370 motor-integrated rear-axle Hall encoder provides pulses to the ESP32; the ESP32 calculates pulse counts and sends them to Jetson, which calculates the final odometry. The micro-ROS Agent on Jetson connects Jetson ROS 2 and the ESP32 micro-ROS node through the W5500 wired link: motor, steering, OLED, notification-tone, and status topics use Micro XRCE-DDS / UDP `8888`, while raw encoder odometry uses independent UDP `9999`. Jetson connects to the W5500 by Ethernet cable, and the W5500 connects to the ESP32 by Dupont jumper wires. All four Jetson USB-A ports are occupied by current peripherals, and its USB-C port is used for debug mode rather than this communication link. The vehicle does not use Wi-Fi under the competition rules.

## Contents

- [System components and responsibilities](#system-components-and-responsibilities)
- [Physical layout](#physical-layout)
  - [Layered arrangement](#layered-arrangement)
  - [Sensor placement and geometry](#sensor-placement-and-geometry)
  - [Drive, steering, and communication-module placement](#drive-steering-and-communication-module-placement)
- [Physical connections, ROS 2 topics, and data flows](#physical-connections-ros-2-topics-and-data-flows)
- [Key architecture decisions](#key-architecture-decisions)
- [Risks, limitations, and mitigations](#risks-limitations-and-mitigations)

```mermaid
flowchart LR
    BAT[12 V main battery and master switch] --> PWR[Controlled power branches]
    PWR --> JET[Jetson Orin Nano\nPerception, state estimation, and decisions]
    START[Program-start button\nJetson physical pin 29] -->|active-low robot_autostart trigger| JET
    PWR --> ESP[ESP32-S3\nActuator-side control]
    PWR --> SNS[LiDAR, depth camera, and IMU]
    SNS --> JET
    ENC[GA25-370 integrated Hall encoder] -->|Pulses| ESP
    JET --> AGENT[micro-ROS Agent\nROS 2 DDS ↔ Micro XRCE-DDS]
    AGENT <-->|Micro XRCE-DDS / UDP 8888\nEthernet cable| W5500[W5500]
    W5500 <-->|Dupont jumper wires| ESP
    ESP -->|Raw odometry\nUDP 9999| W5500
    ESP --> ACT[Drive motor and steering servo]
    ACT --> VEH[Vehicle motion]
    VEH --> SNS
    JET --- LOG[Logging and telemetry]
    ESP --- LOG
```

## System components and responsibilities

| Subsystem | Confirmed component or function | Inputs | Outputs or interfaces | Architecture rationale |
|---|---|---|---|---|
| Main power and first operation | One 12 V competition battery, main power switch, and controlled power branches | Battery energy | Computing, sensing, and control branches | The main power switch is the first permitted operation and supplies power to the complete vehicle; see Figure 3 for its physical location. |
| Program start and second operation | Jetson-GPIO program-start button and `robot_autostart` service | Button pulls Jetson physical pin `29` low | Service starts `planner.launch.py` | The program-start button is the second permitted operation and starts vehicle motion; see Figure 3 for its physical location. |
| Perception | Two STL-27L 2D LiDARs and D435f depth camera | Walls, pillars, parking area, and forward scene | Environment observations for Jetson | Scan data from the two LiDARs are merged for downstream processing; the D435f depth camera provides forward vision and depth/distance information. |
| Vehicle state | TM171 IMU and GA25-370 motor-integrated rear-axle Hall encoder | Heading/attitude and rear-axle pulses | IMU data and pulse counts sent by ESP32 | ESP32 reads and calculates encoder pulses; Jetson calculates final odometry from the received pulse counts. |
| High-level computing | Jetson Orin Nano Super Developer Kit | Sensor data, IMU data, and pulse counts sent by ESP32 | Perception, state estimation, decisions, and control requests | Separates compute-intensive perception/decisions from actuator-side control. |
| ROS communication bridge | micro-ROS Agent on Jetson and micro-ROS node on ESP32 | ROS 2 control, display, and audio topics; ESP32 status | ROS 2 topic bridge between Jetson and ESP32 | The Agent connects Jetson ROS 2 with the ESP32 micro-ROS node; ESP32 firmware then interprets each command and operates hardware. |
| Actuator-side control | ESP32-S3 and carrier board | Control requests from Jetson and integrated Hall-encoder pulses | Drive-motor and steering-servo control; pulse counts sent to Jetson | ESP32 reads and calculates encoder pulses while handling actuator-side control. |
| Between-computer communication | W5500, Jetson-W5500 Ethernet cable, and W5500-ESP32 Dupont jumper wires | Jetson and ESP32 data | Jetson connects to W5500 by Ethernet cable; W5500 connects to ESP32 by Dupont jumper wires | All four Jetson USB-A ports are occupied by current peripherals, its USB-C port is for debug mode, and competition rules do not allow Wi-Fi. |
| Actuation | Rear-drive GA25-370 motor, front SC-1258TG+ steering servo, servo-linkage assembly, and Ackermann front axle | ESP32 control outputs | Longitudinal motion and front-wheel steering | The rear-drive motor controls forward, reverse, and speed; the steering servo drives the Ackermann front axle through the servo-linkage assembly to control front-wheel steering and travel direction. |
| Recording | Jetson/ESP32 logs and telemetry | Perception, decision, and control state | Traceable debugging and test evidence | Supports fault isolation, test comparison, and pre-competition checks. |

## Physical layout

### Layered arrangement

The lowest layer holds steering, rear drive, and battery; the middle layer provides the battery bay and mounting positions for both LiDARs; the upper layer holds the ESP32, Jetson, perception hardware, and cross-layer wiring.

| Lowest layer | Middle LiDAR layer | Top deck |
|---|---|---|
| ![Lowest vehicle layer, including steering, rear drive, and battery position.](../media/physical-layout/layer-01-bottom-chassis.jpg) | ![Middle layer with both LiDARs.](../media/physical-layout/layer-02-lidar.jpg) | ![Top vehicle deck.](../media/physical-layout/layer-03-top-deck.jpg) |

*Figure 1: Layered vehicle structure. The left image shows the lowest layer; the center image shows the middle layer holding the two LiDARs; the right image shows the top deck.*

| Rear view of layers | Upper layer with ESP32 and speaker | Complete layers with Jetson |
|---|---|---|
| ![Rear view of the layered vehicle.](../media/physical-layout/layered-rear-view.jpg) | ![Upper layer with ESP32 and speaker.](../media/physical-layout/top-layer-esp32-speaker.jpg) | ![Side view of the complete vehicle layers after Jetson installation.](../media/physical-layout/layered-vehicle-with-jetson.jpg) |

*Figure 2: Complete layered vehicle arrangement. The left image is a rear view; the center image shows the upper layer holding the ESP32 and speaker; the right image shows the complete side layering after Jetson installation.*

![Top view of the upper electronics after cable routing.](../media/physical-layout/upper-layer-cable-routing.jpeg)

*Figure 3: Wiring of the upper electronics. This image records the Jetson, ESP32, W5500, and their connected cable harnesses.*

**Main power and program start:** The main power switch performs the first operation and powers the complete vehicle. The program-start button is beside Jetson; pressing it pulls physical Jetson pin `29` low, which causes the `robot_autostart` service to launch `planner.launch.py`. Together, these are the first and second operations specified by the competition rules.

| Main power switch | Program-start button location |
|---|---|
| ![Vehicle main power switch.](../media/power-and-start-controls/main-power-switch.jpg) | ![Close-up of the blue program-start button beside Jetson.](../media/power-and-start-controls/program-start-button-closeup.jpeg) |

*Figure 4: Physical switches used for the two operations. The left image shows the main power switch; the right image shows a close-up of the blue program-start button beside Jetson.*

### Sensor placement and geometry

**Dual LiDARs:** The front LiDAR is upright; the rear LiDAR is inverted, longitudinally offset, and raised by a `5 mm` printed spacer. Both fixed horizontal scan planes are below `95 mm` and intersect track objects approximately `100 mm` high.

![Two STL-27L 2D LiDARs for front and rear installation.](../media/final-architecture/lidar-modules.jpg)

*Figure 5: Two STL-27L 2D LiDARs for front and rear installation.*

| Platform before installation and rear-LiDAR spacer | Front-LiDAR mounting holes | Rear-LiDAR mounting holes |
|---|---|---|
| ![Platform before LiDAR installation; the vehicle rear is at top and the rear-LiDAR position has a 5 mm spacer.](../media/final-architecture/lidar-platform-before-installation.jpg) | ![Reserved front-LiDAR mounting holes.](../media/final-architecture/front-lidar-mounting-holes.jpg) | ![Reserved rear-LiDAR mounting holes.](../media/final-architecture/rear-lidar-mounting-holes.jpg) |

*Figure 6: Dual-LiDAR mounting structure.*

| Front view | Top view |
|---|---|
| ![Front view of installed LiDARs.](../media/final-architecture/dual-lidar-front-view.jpg) | ![Top view of installed LiDARs.](../media/final-architecture/dual-lidar-top-view.jpg) |

*Figure 7: Physical installation of the dual LiDARs.*

**D435f depth camera:** Fixed on the vehicle's forward centreline and facing forward.

| Reserved camera mounting holes | Camera installation |
|---|---|
| ![Reserved camera mounting holes at the vehicle front.](../media/final-architecture/d435f-front-mounting-holes.jpg) | ![D435f installation at the front centre.](../media/final-architecture/d435f-installation.jpg) |

*Figure 8: D435f mounting holes and installation.*

![D435f depth camera fixed at the centre of the vehicle front.](../media/final-architecture/73.jpg)

*Figure 9: Final physical installation of the D435f depth camera.*

**TM171 IMU:** Mounted in the lower vehicle body on a raised level beside Jetson.

| IMU mounting position | TM171 IMU unit |
|---|---|
| ![Interior vehicle view with the IMU mounting position marked.](../media/electronics-interfaces/tm171-imu-installation-highlighted.png) | ![Close-up of the TM171 IMU unit.](../media/electronics-interfaces/tm171-imu-closeup.jpg) |

*Figure 10: TM171 IMU mounting position and unit.*

### Drive, steering, and communication-module placement

| W5500 and raised platform before installation | Assembled raised platform | Installed on vehicle |
|---|---|---|
| ![W5500, platform, and fasteners.](../media/electronics-interfaces/w5500-platform-components-before-installation.jpg) | ![W5500 installed on platform.](../media/electronics-interfaces/w5500-platform-assembled.jpg) | ![Platform installed on vehicle.](../media/electronics-interfaces/w5500-installed-on-vehicle.jpg) |

*Figure 11: W5500 and raised-platform installation.*

| GA25-370 motor | Rear-drive installation components |
|---|---|
| ![GA25-370 rear-drive motor with the integrated Hall-encoder lead at top.](../media/final-architecture/rear-drive-motor-closeup.jpg) | ![Rear-drive components before installation.](../media/final-architecture/rear-drive-installation-components.jpg) |

*Figure 12: GA25-370 rear-drive motor and installation components.*

| Timing pulleys and ball bearings | Assembled rear drive |
|---|---|
| ![Timing pulleys and ball bearings.](../media/final-architecture/timing-pulleys-and-ball-bearing.jpg) | ![Motor, pulleys, and bearings after assembly.](../media/final-architecture/rear-drive-motor-pulley-bearing-assembly.jpg) |

*Figure 13: Rear-drive transmission assembly.*

| One transmission side | Other transmission side | Rear-wheel axle |
|---|---|---|
| ![One rear-drive transmission detail.](../media/final-architecture/rear-drive-transmission-detail-01.jpg) | ![Other rear-drive transmission detail.](../media/final-architecture/rear-drive-transmission-detail-02.jpg) | ![Rear-wheel axle.](../media/final-architecture/rear-wheel-axle.jpg) |

*Figure 14: Rear-drive transmission and rear-wheel-axle details.*

| Steering overview | Servo-linkage assembly and servo |
|---|---|
| ![SC-1258TG+, servo-linkage assembly, and Ackermann front axle.](../media/final-architecture/steering-assembly-overview.jpeg) | ![Ball-joint links, steering-knuckle links, and steering servo.](../media/final-architecture/steering-linkage-components.jpeg) |

*Figure 15: Steering-system component overview.*

![SC-1258TG+ steering-servo unit.](../media/final-architecture/steering-servo-closeup.jpg)

*Figure 16: SC-1258TG+ steering-servo close-up.*

| Rear view | Top view | Bottom view |
|---|---|---|
| ![Rear view of installed steering system.](../media/final-architecture/front-axle-servo-rear-view.jpg) | ![Top view of installed steering system.](../media/final-architecture/front-axle-servo-top-view.jpg) | ![Bottom view of installed steering system.](../media/final-architecture/front-axle-servo-bottom-view.jpg) |

*Figure 17: Three views of the installed steering system. The 3D-printed teardrop-shaped servo horn and servo-linkage assembly are installed on the Ackermann front axle.*

![Ackermann front axle while steering.](../media/final-architecture/front-axle-ackermann-turning.jpg)

*Figure 18: Ackermann front axle in a steering state.*

![Physical Ackermann-front-axle steering knuckles.](../media/final-architecture/ackermann-steering-knuckles.jpg)

*Figure 19: The two Ackermann front-axle steering knuckles.*

## Physical connections, ROS 2 topics, and data flows

This section records physical connections, ROS 2 topics or protocols, data flows, and purpose for each component or link.

### 1. Dual-LiDAR scan-data merging

**Physical connections:** The front LiDAR ZHR-4 cable connects directly to Jetson 40-pin GPIO through Dupont jumpers: yellow to `5 V` (Pin 4), white to GND (Pin 9), black TX to Jetson RX (Pin 10), and red PWM unconnected. The rear LiDAR ZHR-4 cable connects to a CP2102 TTL-to-USB-A module: black TX to CP2102 RX, white to GND, yellow to `5 V`, and red PWM unconnected; CP2102 USB-A plugs into Jetson.

| Front-LiDAR ZHR-4 interface | Front LiDAR to Jetson GPIO |
|---|---|
| ![Front-LiDAR ZHR-4 interface.](../media/electronics-interfaces/front-lidar-zhr4-interface.jpeg) | ![Front LiDAR connected to Jetson GPIO through Dupont jumpers.](../media/electronics-interfaces/front-lidar-zhr4-to-gpio-dupont.jpeg) |

*Figure 20: Physical interface and wiring of the front LiDAR.*

| Rear-LiDAR ZHR-4 interface | ZHR-4 to CP2102 RX and USB-A | CP2102 USB-A and RX |
|---|---|---|
| ![Rear-LiDAR ZHR-4 interface.](../media/electronics-interfaces/rear-lidar-zhr4-interface.jpeg) | ![Rear LiDAR cable to CP2102 RX and Jetson USB-A.](../media/electronics-interfaces/rear-lidar-zhr4-cp2102-rx-to-usb-a.jpeg) | ![CP2102 USB-A and RX interfaces.](../media/electronics-interfaces/cp2102-usb-a-rx-interface.jpeg) |

*Figure 21: Physical interface and wiring of the rear LiDAR.*

**ROS 2 topics and data flow:** `/lidar1/scan` + `/lidar2/scan` → merge node → `/scan` → Jetson wall, corridor, and obstacle processing. All three topics use `sensor_msgs/msg/LaserScan`.

![RViz displaying Laser 1, Laser 2, and Merged Laser.](../media/runtime-visualizations/2-lidar-merged-rviz.png)

*Figure 22: RViz verification of dual-LiDAR scan-data merging.*

### 2. D435f depth and color camera inputs

**Physical connection:** D435f uses USB-C and connects to Jetson through a USB-C-to-USB-A cable.

| D435f camera | USB-C-to-USB-A cable |
|---|---|
| ![D435f depth camera.](../media/final-architecture/d435f-camera-closeup.jpg) | ![USB-C at camera end and USB-A at Jetson end.](../media/final-architecture/d435f-usb-c-to-jetson-usb-a-cable.jpg) |

*Figure 23: D435f depth camera and connection cable.*

**ROS 2 topics and data flow:** Color images use `/camera/camera/color/image_raw`; used depth images use `/camera/camera/aligned_depth_to_color/image_raw`. The camera sends forward color and depth/distance information to Jetson.

![D435f DepthCloud RViz runtime view.](../media/runtime-visualizations/depth-camera-rviz.png)

*Figure 24: RViz verification of the D435f depth camera.*

### 3. TM171 IMU input

**Physical connection:** TM171 uses USB-C and connects to Jetson through a USB-C-to-USB-A cable.

| IMU USB-C interface | Installed IMU USB-C-to-Jetson-USB-A connection | IMU USB-C connection for Windows debugging |
|---|---|---|
| ![TM171 USB-C interface.](../media/electronics-interfaces/imu-usb-c-interface-closeup.jpeg) | ![Installed IMU cable connection.](../media/electronics-interfaces/imu-usb-c-to-jetson-usb-a-overview.jpeg) | ![TM171 IMU connected through USB-C for Windows debugging.](../media/electronics-interfaces/tm171-imu-usb-c-windows-debug-connection.jpeg) |

*Figure 25: TM171 IMU USB-C interface, installed cable connection, and Windows-debugging connection diagram.*

**ROS 2 topics and data flow:** TM171 → Jetson → `/imu_data`, providing attitude and heading information.

![RViz Imu display subscribing to `/imu_data`.](../media/runtime-visualizations/imu-rviz.png)

*Figure 26: RViz verification of the TM171 IMU.*

![RViz Mag display subscribing to `/imu_data_mag`.](../media/runtime-visualizations/imu-mag-rviz.png)

*Figure 27: RViz verification of the TM171 magnetometer. This topic is publishing, but the team does not use magnetometer data for control, state estimation, or competition decisions.*

### 4. W5500 Ethernet wired communication interface

**Physical connection:** Jetson connects to W5500 through an orange Ethernet cable; W5500 connects to ESP32 through Dupont jumper wires.

![W5500, Ethernet cable, and Dupont-jumper wiring overview.](../media/electronics-interfaces/w5500-jetson-ethernet-esp32-dupont-overview.jpeg)

*Figure 28: Physical communication overview of the W5500 module.*

| Top view | Right-side view |
|---|---|
| ![Top view of Dupont jumpers connected to ESP32.](../media/electronics-interfaces/w5500-integrated-wiring-01.jpeg) | ![Right-side view of Dupont jumpers connected to ESP32.](../media/electronics-interfaces/w5500-integrated-wiring-02.jpeg) |

*Figure 29: Dupont jumpers connected to the ESP32.*

### 4.1 micro-ROS Agent, status, and human-machine feedback

**Link:** Jetson ROS 2 nodes → micro-ROS Agent on Jetson (Micro XRCE-DDS / UDP `8888`) → Ethernet cable → W5500 → Dupont jumpers → ESP32 micro-ROS node; ESP32 status topics return along the reverse link. The Agent connects Jetson ROS 2 with the ESP32 micro-ROS node, while ESP32 callbacks interpret received topics as hardware actions.

This link carries motor and steering control as well as OLED status, low-battery blue-screen flashing, volume, and start notification tones.

### 5. Rear-drive motor, encoder pulses, and odometry

**Speed-control path:** Jetson control node → micro-ROS Agent → W5500 → ESP32 → GA25-370 rear-drive motor.

**Feedback path:** GA25-370 integrated Hall encoder → ESP32 pulse-count calculation → W5500 → Jetson. `motor-api.ino` registers `CHANGE` interrupts for both encoder pins and increments or decrements `encoder_ticks` from their state changes. `odom_udp.ino` sends `encoder_ticks`, `servo_pos`, and `timestamp_ms` to Jetson UDP `9999` every `50 ms` (`20 Hz`). `odom_udp_node.py` unpacks the data, converts cumulative pulses to distance, then uses steering position and Ackermann kinematics to calculate and publish `/wheel/odometry`.

ESP32 may also publish `/microROS/encoder_data` as a cumulative-distance check; final `/wheel/odometry` uses raw UDP encoder data and can be compared with this value.

### 6. Steering servo, servo-linkage assembly, and Ackermann front axle

**Control path:** Jetson publishes `/microROS/servo_control` → Jetson micro-ROS Agent → W5500 → ESP32 → SC-1258TG+ steering servo → 3D-printed teardrop-shaped servo horn → servo-linkage assembly → Ackermann front axle. ESP32 calls `Ctrl_sg90()` to drive the servo; the linkage transfers servo angle to both front wheels.

### 7. Runtime records and telemetry

Jetson and ESP32 record perception, state, control, and communication status for debugging, test comparison, and pre-competition checks.

## Key architecture decisions

### Steering, chassis, and wheels

- **Changed from parallel steering to positive Ackermann.** Parallel steering could not make the required large, repeatable continuous turn. The team considered the high-speed Anti-Ackermann / Reverse-Ackermann approach, but this vehicle does not need high-speed cornering; the final positive-Ackermann geometry turns the inner front wheel more than the outer wheel and reduces low-speed tyre scrub.
- **Turning envelope set the chassis geometry.** The design target was a theoretical single continuous turn within the `375 mm` course width. V1 body length, wheelbase, and front overhang were reduced to the current geometric basis of a `220 mm` body and `138 mm` wheelbase.
- **7 mm clearance and wheel choice.** Because the course surface is smooth and fixed, the final `7 mm` ground clearance lowers the centre of gravity and keeps vertical space for upper electronics. Front-wheel diameter is approximately `50 mm`; the shared, non-differential rear axle uses smoother `65 mm` rear wheels. Early high-friction `68 mm` commercial wheels dragged and occasionally slipped at high steering angles; after the change, no front ballast was added.
- **Structural material and servo centre.** PLA was retained only for prototypes. After comparing PA12-CF and PA6-CF, the load-bearing lower layers use PA6-CF to preserve steering and belt-transmission geometry; the upper electronics layer remains PLA for electrical insulation. When the installed servo horn centre did not match the default servo pulse centre, and an external resetter lacked sufficient power, Python was used to command the default centre pulse.

### LiDAR, camera, and layered layout

- **LiDAR scan height determined the layer arrangement.** Both LiDAR scan planes are below `95 mm`, so they intersect approximately `100 mm` track walls and obstacles. The front LiDAR is upright; the rear LiDAR is inverted, longitudinally offset, and raised by a `5 mm` spacer. This geometry determined the final locations of battery, LiDARs, and electronics layers.
- **Merged 2D scans.** Independent front and rear `LaserScan` data are merged and published as `/scan` for wall, corridor, and obstacle processing.
- **The depth camera did not retain YOLO as the final approach.** D435f depth is aligned with color for red/green obstacle recognition and distance association. The team tried YOLO training, but recognition was unstable with the available training dataset, so the final control path does not depend on YOLO.

### Power, installation, and human-machine feedback

- **Changed from two batteries to one 12 V main battery.** Early designs allocated separate power to Jetson (SBC) and ESP32/actuators (SBM) and considered a 12 V-to-19 V Jetson boost branch. Against the two-operation rule, mass, and the [component power and current assessment](05-power-and-sensor-architecture.md#component-power-and-current-assessment), the final design uses one `12 V` battery. That sizing estimate records approximately `2.3 A` operating current and `7.6 A` short-peak current; the boost module was removed because of its mass.
- **Battery installation and state display were unified.** Magnetic mounting was rejected because it could only attach on the prohibited reverse face; the battery uses a low-level tray and screws. With one battery, the ESP32-side percentage display, low-battery alert, and startup-volume cue are retained instead of maintaining two state displays.
- **Power wiring and switch adapter.** The component assessment judged a `16 AWG` main lead sufficient, but the final installed main wire is the more conservative `14 AWG`; the two branches are `18 AWG`. Testing showed that the DC5525 plan was not a reliable conducting battery interface, so the battery-to-branch interface uses verified DC5521; the Jetson branch is DC5525 and the ESP32 branch is DC5521. The plastic main-power switch is not soldered directly; it uses flag-terminal adapters.

![Flag-terminal adapter at the main power switch.](../media/power-and-start-controls/main-power-switch-flag-terminal.jpg)

*Figure 30: Flag-terminal adapter at the main power switch. The adapter avoids direct soldering to the plastic switch terminals.*

### Jetson, ESP32, and wired communication

- **ROS 2 / micro-ROS division of work.** Jetson handles perception, state estimation, decisions, and ROS 2 control nodes; ESP32 handles actuator-side control and encoder-pulse acquisition. The micro-ROS Agent on Jetson bridges ROS 2 topics to the ESP32 micro-ROS node over Micro XRCE-DDS / UDP `8888`; raw encoder odometry uses independent UDP `9999`.
- **Changed from Wi-Fi/USB candidates to W5500.** Competition rules do not allow Wi-Fi. A driverless USB Ethernet adapter was considered, but ESP32 had no matching driver; the final choice is ESP32-supported W5500. The original module placed header pins and Ethernet port on the same side, leaving it vulnerable, so the header was changed to the reverse side and a raised platform was designed. A lower-right-angle-to-straight Ethernet cable connects to Jetson along a protected route.

### Software architecture trade-off

- **Common-library SLAM and Nav2 are not the final course solution.** The team connected LiDAR/camera, IMU, and odometry EKF localization to `slam_toolbox`, but the map kept jumping. It also tried `RTAB-Map` 3D SLAM, which proved less stable than the 2D SLAM attempt. Nav2 was then tried; in a `375 mm` course, small obstacles and Nav2's avoidance-inflation space did not fit the vehicle's narrow-corridor requirements.
- **Self-built map, localization, and planning chain.** The team implemented its own map and localization and uses planner code for planning. The vehicle uses merged `/scan`, IMU, depth-camera, and encoder information to manage local state and plan its motion. The team also maintains its own simulation environment.

## Risks, limitations, and mitigations

| Risk or limitation | Current condition | Mitigation in use |
|---|---|---|
| Camera USB connection | No `10 cm` USB 3.0-or-higher right-angle-to-straight cable was found that fits between the camera and Jetson; the current camera connection produces a speed warning. Longer cables could drag on obstacles and were not adopted. | Keep the current short cable route; check camera topics and depth output before a run. |
| Inverted-LiDAR cable | When the battery is removed and the vehicle is turned over, the relatively loose LiDAR cable can cause a LiDAR not to operate. | At every startup, confirm both LiDARs are lit and verify both independent scans and the merged scan in RViz. |
| Depth-camera mounting height | The camera is mounted high enough that nearby objects can fall outside its effective field of view. | LiDAR supplies the current near-range environment information. A later mechanical revision should tilt the forward camera mount downward. |
| Program-start button | A press is emitted once, but releasing the button does not emit a release; its state remains pressed. | `robot_autostart` ignores subsequent button presses while the planner is already running; release handling still needs to be completed in the input logic. |
| micro-ROS Agent restart | The micro-ROS Agent occasionally restarts; its previous communication state is then lost. | Timeout and watchdog handling were added, and one multiply used pin was removed. These changes reduced restart frequency but did not eliminate restarts. |
| Dual-LiDAR scan overlap and occlusion | The two LiDAR scan areas have overlap and failure locations; four brass standoffs and cable harnesses may affect local scan stability. | Custom map/state management retains prior observations to reduce the effect of a single local loss; wiring remains inside the vehicle outline and away from scan planes. |
