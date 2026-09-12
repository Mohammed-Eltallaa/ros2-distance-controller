# Distance Controller — ROS 2 Mobile Robot

A mobile robot built on a Raspberry Pi 4 running ROS 2 Humble that maintains a safe distance from obstacles using ultrasonic sensing, with an experimental LiDAR-based object-tracking mode. Built as a capstone project for the Lab Electronic Systems course at Hochschule Heilbronn.

## Overview

The original goal was a robot that could follow a moving object while keeping a safe distance, fusing ultrasonic and LiDAR data. Hardware limitations with the LiDAR sensor (see [LiDAR Issues](#lidar-issues) below) made reliable object tracking infeasible within the project timeline, so the design was pivoted to a fully working **autonomous obstacle-avoidance system** driven primarily by ultrasonic input, with LiDAR contributing a simple "path blocked" safety flag.

The codebase has two versions:

- **`original`** — the initial distance-following implementation, later adapted into the anti-collision system that runs reliably.
- **`2ndLidarAttempt`** — a second implementation that adds a dedicated LiDAR object-tracking node. Not fully functional due to suspected LiDAR hardware issues, but left in the repo for future groups to continue.

## Hardware

- Raspberry Pi 4 Model B (Ubuntu 22.04 LTS)
- HC-SR04 ultrasonic distance sensor
- 2D LiDAR (RPLiDAR-class, via `hlds_laser_publisher`)
- L298N dual H-bridge motor driver
- 2x DC motors, differential drive chassis (custom 3D-printed, designed in SolidWorks, printed in PLA on a Raise3D N2)
- LiPo battery + DC-DC converter (custom-printed mounts)

## Software architecture

Built entirely on ROS 2's publisher/subscriber model — each responsibility (sensor polling, filtering, control, actuation) lives in its own node, so a failure or missing subsystem (e.g. LiDAR) doesn't take down the rest of the robot.

**Data flow (`original` / working obstacle-avoidance mode):**

/ultrasonic_driver → /distance_raw → /distance_filter → /distance_filtered
↓
/hlds_laser_publisher → /scan → /lidar_steering_node → /path_blocked
↓
/pid_distance_controller → /cmd_vel → /motor_node


### Node breakdown

| Node | Subscribes | Publishes | Role |
|---|---|---|---|
| `ultrasonic_driver` | — | `/distance_raw` | Triggers the HC-SR04, times the echo, computes distance |
| `distance_filter` | `/distance_raw` | `/distance_filtered` | Validates range (3–380 cm) and applies a 3-sample median filter to reject noise spikes |
| `lidar_steering_node` | `/scan` | `/path_blocked` | Slices the LaserScan to a front-facing 90° cone, flags a boolean if the path is blocked within 50 cm |
| `pid_distance_controller` | `/distance_filtered`, `/path_blocked` | `/cmd_vel` | Central controller — runs a PID loop to hold a setpoint distance in normal operation, and switches to an escape/rotation state machine when blocked |
| `motor_node` | `/cmd_vel` | — | Converts `Twist` velocity commands to per-wheel PWM/direction via GPIO, driving the L298N |

**Additional nodes in `2ndLidarAttempt`:**

| Node | Role |
|---|---|
| `object_tracker` | Detects the closest object in a 30° front cone from `/scan` and publishes a proportional steering command on `/cmd_vel_angle`. Not fully validated — see LiDAR Issues. |
| `velocity_combiner` | Merges the PID's linear velocity (`/cmd_vel_distance`) with the LiDAR tracker's angular velocity (`/cmd_vel_angle`) into a single `/cmd_vel` command |

### Control logic

- **Filtering:** raw ultrasonic readings are noisy and prone to single-frame spikes (e.g. `[15, 14, 800, 16, 15]`). A moving average is thrown off badly by outliers, so a 3-sample rolling **median filter** is used instead — it removes spikes while preserving real changes in distance.
- **PID controller:** maintains a setpoint distance (20 cm) from the nearest obstacle using standard P/I/D terms with integral anti-windup. Gains used: `Kp = 0.22`, `Ki = 0.001`, `Kd = 0.02`.
- **Safety state machine:** if the LiDAR flags a blockage or the ultrasonic reading drops below ~15 cm, the robot switches into an escape mode (in-place rotation) until both sensors report a clear path, then resumes normal PID-driven tracking.

## LiDAR Issues

Integrating the LiDAR was the most technically demanding part of the project:

- Custom driver required from course staff; `udev` rules were needed to stop Linux from reassigning the sensor's USB port on every reboot.
- Parsing the full 360-point `LaserScan` array in Python at 10 Hz overloaded the Pi's CPU, adding ~2 seconds of latency. This was mitigated by slicing to a 90° front cone and switching QoS from `RELIABLE` to `BEST_EFFORT`.
- During diagnostics, the sensor consistently returned valid readings only in the 225°–360° sector — the front half of the scan (0°–225°) returned no data at all, regardless of software fixes or physically rotating the sensor 90°. This pointed to a hardware fault rather than a software bug.
- As a result, the `object_tracker` node (LiDAR-based following) could not be fully validated. Only the ultrasonic distance-following / obstacle-avoidance path was verified end-to-end.

A diagnostic snippet for reproducing this sector test is included in the code documentation for future groups debugging the same sensor.

## Results

- The median filter completely eliminated high-frequency noise spikes from the raw ultrasonic signal, giving smooth PID-driven motor responses.
- The robot reliably navigated hallways and indoor environments (carpet, tile, tight corridors), stopping and rotating away from walls and obstacles as expected.
- Minor drift to one side during long straightaways was observed, attributed to manufacturing differences between the two DC motors (the robot has no wheel encoders / closed-loop odometry).
- The ROS 2 modular architecture meant the LiDAR subsystem could be disabled entirely without affecting the ultrasonic-based obstacle avoidance — demonstrating good separation of concerns.

## Future work

- Add wheel encoders for closed-loop odometry and straight-line accuracy.
- Investigate/replace the LiDAR unit to fix the front-sector blind spot, then re-enable `object_tracker`.
- Once LiDAR is reliable, revisit SLAM-based navigation.

## Getting started

```bash
# On the Raspberry Pi (Ubuntu 22.04 + ROS 2 Humble):
cd ~/ros2_ws
colcon build
source install/setup.bash

# Run the working obstacle-avoidance stack:
ros2 run <package_name> ultrasonic_driver
ros2 run <package_name> distance_filter
ros2 run <package_name> lidar_steering_node
ros2 run <package_name> pid_distance_controller
ros2 run <package_name> motor_node
```

> Adjust package/executable names to match your `setup.py` / `package.xml` once the code is organized into a colcon package.


## References

- [ROS 2 Humble Hawksbill Documentation](https://docs.ros.org/en/humble/index.html)
- [Raspberry Pi 4 Model B Documentation](https://www.raspberrypi.com/documentation/)
- [Ubuntu 22.04 LTS Release Notes](https://ubuntu.com/documentation)
- SparkFun HC-SR04 Ultrasonic Distance Sensor Datasheet, Rev. 1.4
- Slamtec RPLiDAR A1/A2 User Manual and ROS Integration Guide
- P. Corke, *Robotics, Vision and Control*, Springer, 2011
- R. Siegwart, I. R. Nourbakhsh, D. Scaramuzza, *Introduction to Autonomous Mobile Robots*, 2nd ed., MIT Press, 2011
