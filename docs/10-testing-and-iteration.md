# Testing and Iteration

## Purpose

Connect the vehicle's mechanical, electrical, sensing, communication, and software iterations to the recorded calculations, observations, and tests. This chapter indexes evidence already documented in Chapters 01–05; it does not present an unmeasured design claim as a physical test result.

## Summary

The current vehicle was reached through linked iterations. Mechanical work changed steering geometry, chassis envelope, rear wheels, support height, and cable routes. Electrical work changed the two-battery concept into one protected 12 V system and removed the 19 V boost module. Sensor work fixed the LiDAR scan planes relative to the `100 mm` course objects, replaced an unresponsive LiDAR cable, and established a common vehicle frame. Communication and control work selected W5500 wired Ethernet and the team's own planning chain after alternatives did not fit the hardware or narrow course.

## Questions addressed

- Can V3 geometry and positive Ackermann fit the intended `375 mm` turn space better than the earlier parallel-steering design?
- Does the GA25-370 rear drive provide sufficient acceleration and speed while a non-differential rear axle remains controllable?
- Can one 12 V battery, direct Jetson input, and the installed harness operate the two-controller vehicle?
- Do the LiDAR positions intersect the `100 mm` course objects and provide usable merged front/rear coverage?
- Which communication, micro-ROS, localization, and planning choices are compatible with the ESP32, Jetson, and narrow course?

## Test and evidence boundaries

The entries distinguish **calculation** (results derived from stated inputs), **vehicle test or observation** (recorded physical outcomes), and **integration verification** (for example, RViz data visibility or successful firmware deployment). Remaining limitations are retained as limitations.

## Tests and results

| Test ID | Question or hypothesis | Setup and metric | Recorded result | Conclusion and next action |
|---|---|---|---|---|
| T-MECH-01 | V3 and positive Ackermann reduce turning demand compared with V1. | Compare V1/V2/V3 dimensions and calculate the V3 rear-axle-centre radius from the `138 mm` wheelbase and `53.7°`/`33.7°` design angles. | V1 required `561–581 mm` in the recorded turn-width calculation. V3 has a `220 mm` body and `138 mm` wheelbase; its theoretical rear-axle-centre radius is about `154.1 mm`. | V3 is the geometric basis. A full swept-envelope test in the `375 mm` space remains the required physical validation. [Mechanical calculation and result](04-mobility-and-mechanical-design.md#tests-and-results) |
| T-MECH-02 | The selected motor and belt reduction meet the low-speed launch target. | Five `0 → 150` straight-line tests on the flat WRO mat with the `4500 mAh` battery; compare measured acceleration/speed with the torque and speed calculation. | Mean `10%–90%` acceleration was `0.545253 m/s²` (sample SD `0.063005 m/s²`); mean stable speed was `0.319176 m/s`. Calculated rear-axle starting torque was about `0.0466 N·m`. | GA25-370 with the `20T:40T` belt reduction covers the selected launch and speed target. [Evidence and calculation](04-mobility-and-mechanical-design.md#tests-and-results) |
| T-MECH-03 | Smoother printed rear wheels reduce shared-axle drag. | Compare large-angle arc driving with early high-friction `68 mm` wheels and final no-silicone `65 mm` printed wheels. | The early wheels caused substantial drag and occasional slip. Arc driving became more stable after the wheel change. | Retain the `65 mm` wheels; a differential-free axle cannot eliminate drag completely. [Wheel iteration](04-mobility-and-mechanical-design.md#4-tyre-drag-trade-off-for-the-shared-rear-axle-without-a-differential) |
| T-SENSE-01 | Two 2D LiDARs detect `100 mm` course objects and improve coverage when merged. | Install both scan planes at approximately `85 mm`, below `95 mm`; inspect independent scans and merged `/scan` in RViz. | The scan planes intersect walls, signs, and parking limits. Merge gives intended front/rear coverage; standoffs and looms leave local blank returns. | Retain the dual-LiDAR layout and treat occlusion as an active limitation. [LiDAR merge](03-system-architecture.md#1-dual-lidar-scan-data-merging) · [geometry](05-power-and-sensor-architecture.md#assessment-of-field-geometry-and-sensor-installation-positions) |
| T-SENSE-02 | Inverting the IMU to retain its cable inside the vehicle does not break steering behaviour. | Rotate TM171 `180°` from the cable-out-the-rear orientation, then observe automatic steering. | No steering-direction error was observed; automatic steering remained correct. | Retain the inverted installation and protected cable path. [IMU iteration](05-power-and-sensor-architecture.md#imu-orientation-and-cable-routing-iteration) |
| T-POWER-01 | One battery can supply both controllers without boost-converter mass. | Component power/current assessment and direct `12 V` Jetson test after reviewing the `12 V → 19 V` candidate. | Operating total is `27.12 W / 2.26 A`; short-duration wire-sizing value is `7.6 A`. The program ran stably in most cases from the direct 12 V supply. | Retain one `12 V / 4.5 Ah` battery, direct Jetson DC5525 input, `14 AWG` main lead, and `18 AWG` branches. [Power assessment](05-power-and-sensor-architecture.md#component-power-and-current-assessment) |
| T-COMM-01 | The selected SBM can run micro-ROS and maintain a wired Jetson connection. | Attempt ESP-IDF-component deployment, then deploy with `micro_ros_arduino`; use W5500 between controllers. | `micro_ros_espidf_component` conflicted with the board's ESP-IDF 5.4 environment. `micro_ros_arduino` successfully ran on the ESP32; W5500 became the supported link. | Retain ESP32-S3 + W5500 + `micro_ros_arduino`; monitor Agent restarts with timeout/watchdog handling. [Deployment decision](05-power-and-sensor-architecture.md#why-esp32-s3-was-selected-as-the-sbm) |
| T-SW-01 | Generic SLAM/navigation and EKF packages can serve the narrow course. | Evaluate `slam_toolbox`, RTAB-Map, Nav2, and `robot_localization` against the sensor inputs and `375 mm` course. | SLAM maps jumped; RTAB-Map was less stable; Nav2 inflation did not fit the corridor; EKF output was too noisy. | Use the team-built map, localization, and planner chain with encoder-pulse odometry. [Software trade-off](03-system-architecture.md#software-architecture-trade-off) |

## Iteration record

| Iteration | Problem or constraint | Change | Evidence and outcome |
|---|---|---|---|
| Steering and chassis: V1 → V3 | Parallel steering required corrections and V1 exceeded the target turn space. | Positive Ackermann and V3 geometry replaced it. Dimension comparison, radius calculation, and planner-version corner behaviour support the decision; physical swept-envelope evidence remains open. |
| Rear wheels | The shared rear axle dragged and slipped with high-friction wheels. | `68 mm` commercial wheels were replaced by smoother `65 mm` printed wheels; arc driving became more stable. |
| LiDAR cable and rear connection | The `30 cm` ZH1.5-4P-to-USB-A cable produced no LiDAR response; rear LiDAR needed a serial path. | Final `20 cm` ZHR-4-to-Dupont harnesses were used, with a `5.5 cm` CP2102 TTL-to-USB adapter at the rear. |
| Battery, voltage, and harness | Two batteries and a boost module increased mass, wiring, and start complexity. | One protected `12 V / 4.5 Ah` battery, direct 12 V, and a hand-soldered `14 AWG` main / `18 AWG` branch harness became final. |
| Jetson/IMU support | The early `30 mm` support left the IMU very hot and lacked an external start-button mount. | Clearance was raised to `40 mm`; the final support adds the program-start button. |
| Controller transport and planning | ESP-IDF deployment conflicted; Wi-Fi/USB candidates did not fit. Generic SLAM/navigation was unstable or unsuitable. | The final solution uses `micro_ros_arduino`, W5500 Ethernet, and team-built mapping, localization, state management, and planner code. |

## Regression and pre-run checks

1. Confirm the main switch disconnects the vehicle and the separate program-start button starts the planner only after power-on.
2. Check OLED battery state; stop/recharge before the `9%` blue-flash warning becomes a run-time issue.
3. Verify polarity, connector seating, and internal cable routing; no cable may enter moving, sensing, or external-obstacle zones.
4. Verify both LiDARs are lit; inspect each independent scan and merged `/scan` in RViz.
5. Verify D435f, IMU, encoder direction, and W5500/micro-ROS data paths before motion.
6. After repeated runs inspect belt tension/alignment, wheels, supports, sensor mounts, cable routes, and the upper PLA electronics layer.

## Evidence links

- [Team process and iteration approach](01-team-and-project.md)
- [Requirements and competition constraints](02-requirements-and-constraints.md)
- [System architecture, data flows, and software trade-offs](03-system-architecture.md)
- [Mechanical calculations, tests, and iterations](04-mobility-and-mechanical-design.md)
- [Power, interfaces, calibration, and sensor iterations](05-power-and-sensor-architecture.md)
- [Component power and current CSV](../hardware/electronics/power-budget.csv)
- [Component mass CSV](../hardware/mechanical/calculations/weight-budget.csv)

## Open items

- Record the full swept-envelope test in the representative `375 mm` course location, including repeated-run success count and minimum clearance.
- Continue recording micro-ROS Agent restarts and the effect of watchdog/timeout handling.
- Add dated photographs or video links for each final pre-run inspection and regression check.
