# Risks and Failure Modes

## Purpose

Record the vehicle's identified mechanical, electrical, sensing, communication, software, and operating risks. Every entry gives a detection method, mitigation already present in the vehicle, and the remaining limitation or verification action.

## Summary

The vehicle does not depend on Wi-Fi, a boost converter, an exposed cable, a camera-only obstacle decision, or an unprotected battery installation. Some risks are reduced by design; others remain inherent to the current package, including shared-rear-axle drag, planar-LiDAR occlusion, a high camera position, and occasional micro-ROS Agent restart. The correct response is inspection and verification before each run, not claiming that those risks have disappeared.

## Risk register

| ID | Risk / failure mode | Detection | Mitigation in the current vehicle | Verification or remaining limitation |
|---|---|---|---|---|
| R-01 | The shared rear axle has no differential, so high steering angles can create tyre drag or slip. | Observe large-angle arc driving and inspect tyre behaviour. | Positive Ackermann and smoother `65 mm` printed rear wheels reduce scrub. | Arc testing became more stable, but drag cannot be eliminated completely without a differential. |
| R-02 | Low `7 mm` clearance can cause occasional ground contact. | Inspect the underside and observe travel on the fixed smooth course. | Keep low centre of mass and LiDAR scan height; selected rear-drive output can continue over occasional contact. | This is a deliberate trade-off; inspect the chassis after runs. |
| R-03 | Belt tension, pulley alignment, or fasteners can change and reduce rear-drive reliability. The previous year's plastic transmission gear between motor and bearing repeatedly wore and failed five times, and replacement parts could no longer be purchased. | Rotate the drivetrain by hand; inspect belt tracking, pulley alignment, and fasteners before power-on. | Motor slots and adjustment screws set tension; the critical motor-to-rear-axle transmission uses metal timing pulleys and a timing belt instead of a plastic gear that cannot be reliably replaced. | Repeat inspection after transport and repeated runs. |
| R-04 | Carbon-fibre-filled material can conduct near electronics, while PLA can bend under sustained load. | Inspect board clearance and layer condition after runs. | PA6-CF is restricted to lower load-bearing layers; the upper electronics layer is PLA. | Slight upper PLA bend has not affected operation; retain post-run inspection. |
| R-05 | A battery, connector, or harness fault can stop the vehicle or damage electronics. | Check OLED percentage, connector seating, polarity, continuity, and branch routing before power-on. | One protected `12 V / 4.5 Ah` battery, screw-secured bay, main switch, `14 AWG` main/`18 AWG` branches, and internal cable routing. OLED flashes blue below `9%`. | Recharge before low-battery operation; inspect the hand-soldered split harness and flag terminals. |
| R-06 | Cable movement can contact obstacles, steering, rear wheels, or timing belt. | Inspect the complete route; move steering and wheels by hand before a run. | Route all power, sensor, Ethernet, motor, and servo leads through printed-layer openings inside the body outline. | The inverted-LiDAR cable and camera USB cable are mandatory startup inspection points. |
| R-07 | LiDAR can fail because of cable/connector disturbance; standoffs and looms can block parts of the horizontal scan planes. | Confirm both LiDARs are lit; inspect each scan and merged `/scan` in RViz. | Final `20 cm` ZHR-4-to-Dupont leads replaced the unresponsive USB-A lead; front/rear scans are merged and state management retains prior observations. | Physical occlusion still produces local blank returns. |
| R-08 | D435f can lose an object at very close range because its mount is above `100 mm` and it is not a fisheye camera. | Observe the camera view when the vehicle is approximately `20 mm` from an object. | Use LiDAR distance together with camera colour recognition. | `15°` and `20°` camera-mount variants were made but not installed because they require new extrinsic calibration. |
| R-09 | IMU cable routing or orientation can create mechanical drag or incorrect motion interpretation. | Inspect cable path and observe automatic steering behaviour after installation. | Mount TM171 inverted by `180°` to keep its cable inside the vehicle; establish its transform in the URDF vehicle frame. | Automatic steering remained correct in the recorded observation. |
| R-10 | Program-start input or micro-ROS communication can behave unexpectedly during a run. | Check that one press starts the planner; observe Agent status and communication timeouts. | `robot_autostart` ignores repeated presses after startup; timeout and watchdog handling reduce Agent-restart frequency. | Button-release handling and occasional Agent restarts remain open limitations. |
| R-11 | Generic SLAM, Nav2, or EKF output can give unstable state information in the narrow course. | Observe map stability, obstacle clearance, and state output while evaluating the packages. | Use team-built map, localization, state management, and planner code; derive wheel odometry from encoder pulses. | The current approach is selected for this vehicle/course; continued course testing is required. |

## Operational safety sequence

1. Inspect the screw-retained battery bay, main switch, flag terminals, and split harness before connecting power.
2. Confirm all cables are internal, seated, and clear of moving and sensing areas.
3. Check OLED battery state; do not begin a run after the low-battery warning.
4. Power the vehicle once with the main switch, then use the external program-start button as the separate permitted program operation.
5. Before motion, verify LiDAR, camera, IMU, encoder, and W5500/micro-ROS data paths.
6. After a run, inspect belt, wheels, printed layers, sensor supports, cable routes, and connectors for movement or damage.

## Evidence links

- [Requirements and rule-derived constraints](02-requirements-and-constraints.md)
- [System risks, limitations, and mitigations](03-system-architecture.md#risks-limitations-and-mitigations)
- [Mechanical risks, tests, and reproduction checks](04-mobility-and-mechanical-design.md#risks-limitations-and-mitigations)
- [Electrical safety, sensor placement, and failure points](05-power-and-sensor-architecture.md#electrical-and-wiring-safety-measures)
- [Testing and iteration index](10-testing-and-iteration.md)

## Open items

- Record repeated full-course runs with dated logs/video and report failures separately from successful runs.
- Complete the program-start button release-state handling.
- Continue monitoring micro-ROS Agent restarts and LiDAR/camera coverage limitations after each hardware change.
