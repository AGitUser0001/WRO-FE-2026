# Parking Strategy: development scope not completed this season

## Scope conclusion

Parking is not an implemented capability of the final autonomous run this season. Within the available development time, the team completed Jetson/ESP32 communication, sensor integration, manually constructed extrinsic frames, the obstacle-free Open Challenge run, and the Obstacle Challenge's localisation, grids, colour passing, and three-lap logic. There was not enough time to design, tune, and validate parking-area detection and a parallel-parking manoeuvre on the physical vehicle. This repository therefore does not present parking-area localisation, a parking state machine, or a parallel-parking result as completed work.

After the required lap count, the final runtime stops actuator output through `course_complete`; it does not continue to parking search or a parking manoeuvre. This boundary matches the existing planner's lap-completion logic. [Lap confirmation and stop](https://github.com/AGitUser0001/WRO-FE-2026/blob/main/src/robot/planner/planner.py#L2096-L2153)

## Why development stopped here

Parking is not simply a fixed reverse command added after three laps. It would need repeatable entry, reversing, alignment, and stopping while vehicle pose and heading already contain error and while course walls, parking boundaries, and the complete swept vehicle envelope are all present. Adding unvalidated geometric constants to the competition route without enough physical-vehicle tuning time could have damaged the already completed obstacle-course loop.

The team therefore prioritised the integrity of the implemented chain:

1. Maintain continuous pose from odometry and IMU, with limited LiDAR-wall correction;
2. Convert scans into a local grid and fuse them into a global grid;
3. Associate camera red/green recognition with LiDAR distance to choose the passing side;
4. Use weighted path search, corner states, and safe reverse recovery for the three-lap Obstacle Challenge;
5. Stop after three laps rather than claim an unvalidated parking action.

This development sequence and the move from generic SLAM/Nav2 to a dedicated planner are documented in [Chapter 06: Software and Control Architecture](06-software-and-control-architecture.md). The implemented obstacle states, recovery behaviour, and test metrics are in [Chapter 08: Obstacle Challenge Strategy](08-obstacle-challenge-strategy.md).

## Parking work excluded from the final capability

The following work was not completed to a level that can be claimed for the physical vehicle:

| Parking subsystem | Engineering work required | Status this season |
| --- | --- | --- |
| Parking-area recognition | Define LiDAR/camera geometric evidence, coordinate frames, and false-detection handling for the parking area. | No final detector. |
| Entry and parallel manoeuvre | Define entry targets, forward/reverse arcs, and termination conditions from wheelbase, overhangs, track width, and usable clearance. | No final state machine. |
| Safety constraints | Add walls, parking bounds, obstacles, and the vehicle swept envelope to collision checking for the parking path. | No physical-vehicle tuning. |
| Validation | Record repeat-trial success rate, final-position error, contact/line events, and recovery behaviour. | No physical-vehicle results to report. |

## Effect on the competition record

- The Obstacle Challenge video and strategy chapter demonstrate only the recorded three laps, obstacle passing, corners, and safe-recovery behaviour; they are not evidence of completed parking.
- The README and challenge overview identify parking as a competition task and a scope limitation for this season, not as a completed autonomous action.
- If parking is added in later development, its sensing evidence, state transitions, geometric parameters, physical tests, and failure recovery must be documented separately; this chapter cannot substitute for that evidence.

## Related implementation and evidence

- [Software and Control Architecture: development sequence and final-system boundary](06-software-and-control-architecture.md)
- [Open Challenge Strategy: obstacle-free branch of the shared planner](07-open-challenge-strategy.md)
- [Obstacle Challenge Strategy: implemented states, recovery, and metrics](08-obstacle-challenge-strategy.md)
- [Mobility and Mechanical Design: vehicle geometry, steering, and swept envelope](04-mobility-and-mechanical-design.md)
