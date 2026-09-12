#!/bin/bash

trap 'echo -e "\nExecuting Emergency System Shutdown Sequence..."; kill_all_nodes' SIGINT SIGTERM EXIT

kill_all_nodes() {
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -15 "$pid" 2>/dev/null
        fi
    done
    pkill -f motor_controller 2>/dev/null
    pkill -f hlds_laser_publisher 2>/dev/null
    echo "All nodes completely cleaned from background memory."
}

cd ~/ros2_ws
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source install/local_setup.bash

echo "Launching 6 Core Autonomous Navigation Nodes..."
PIDS=()

# 1. Drivers
ros2 run hls_lfcd_lds_driver hlds_laser_publisher --ros-args -p port:=/dev/ttyUSB0 -p frame_id:=laser &
PIDS+=($!)
ros2 run motor_controller driverUltra_node &
PIDS+=($!)

# 2. Filtering & Slicing
env PYTHONUNBUFFERED=1 ros2 run motor_controller median_node 2>&1 | awk 'NR % 10 == 0' &
PIDS+=($!)
ros2 run motor_controller lidar_steering_node &
PIDS+=($!)

# 3. Decision Brain & Output
env PYTHONUNBUFFERED=1 ros2 run motor_controller PIDtest_node 2>&1 | awk 'NR % 10 == 0' &
PIDS+=($!)
ros2 run motor_controller motor_node &
PIDS+=($!)

echo "System fully online. Press [CTRL+C] to exit."
wait

