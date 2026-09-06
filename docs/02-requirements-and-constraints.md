# Requirements and Constraints

## Purpose

Translate competition rules and project constraints into concrete requirements for the vehicle's design, perception, control, and testing.

## Summary

This chapter records the competition rules that directly shaped the vehicle: obstacle colours and passing directions, the three-minute course, the two-operation limit, size and mass limits, the prohibition of differential-drive bases and caster wheels, and required course-object colours. Together, these rules determine the camera, chassis, drive, power, and control approaches.

## Requirements or questions

1. **Red and green obstacles require different passing directions.** Green obstacles are passed on the left and red obstacles on the right. A 2D LiDAR provides distance but cannot distinguish obstacle colour, so the vehicle needs a camera to identify red and green obstacles.
2. **Each competition round lasts three minutes.** The team used this duration to estimate required average speed, starting acceleration, and the resulting drive torque, which guided motor, transmission, and wheel selection.
3. **Only two physical interactions with the vehicle are allowed during a run.** The first turns on vehicle power and the second starts the program; this creates the design requirement for a master power switch and a separate program-start button.
4. **The vehicle may not deliberately leave additional parts or irreversible marks, such as paint, on the course.** The vehicle therefore does not use common line-following modules or a strategy that leaves marks on the course.
5. **Vehicle envelope and mass are limited.** The vehicle may not exceed `300 mm × 200 mm × 300 mm` or `1.5 kg`; these limits constrain chassis dimensions, sensor count, material selection, and battery placement.
6. **A wall collision or moving an obstacle completely out of its designated position fails the round.** The vehicle must maintain obstacle-avoidance and steering accuracy around walls, traffic signs, and parking restrictions instead of relying on contact.
7. **Caster wheels, spherical casters, and ball wheels are prohibited.** Although purchased kits may be assembled for the competition, this rule excludes many off-the-shelf robot chassis that use casters.
8. **Electronic differential drive with one motor per side, such as a differential-wheeled robot, is prohibited.** This also excludes many individually driven off-the-shelf robot cars; this vehicle instead uses front-wheel steering and one motor driving a shared rear axle.
9. **Course-object colours are specified.** Red traffic signs are RGB `(238, 39, 55)`, green traffic signs are RGB `(68, 214, 44)`, and parking restrictions are magenta RGB `(255, 0, 255)`. These colours are direct inputs to camera recognition, the course model, and test-object colour settings.
