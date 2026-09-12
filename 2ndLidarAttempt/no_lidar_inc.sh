#!/bin/bash

# Clear trap to handle emergency shutdown sequence cleanly
trap 'echo -e "\nExecuting Emergency System Shutdown Sequence..."; ym_kill_all_nodes' SIGINT SIGTERM EXIT

ym_kill_all_nodes() {
    # Block recursive traps during cleanup
    trap - SIGINT SIGTERM EXIT
    
    echo "Stopping all tracked background nodes..."
    for ym_pid in "${ym_pids[@]}"; do
        if kill -0 "$ym_pid" 2>/dev/null; then
            kill -15 "$ym_pid" 2>/dev/null
        fi
    done
    
    # Backup cleanup for safety
    pkill -f motor_controller 2>/dev/null
    pkill -f hlds_laser_publisher 2>/dev/null
    echo "All nodes completely cleaned from background memory."
    exit 0
}

cd ~/ros2_ws

# 1. Build first. If C++ code changed, this compiles it.
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source install/local_setup.bash

echo "Launching Core Autonomous Navigation Nodes..."
ym_pids=()

echo "Nodes worden opgestart..."

# ---------------------------------------------------------
# 2. Start alle nodes APART op en sla de PIDs op
# ---------------------------------------------------------

# Hardware Node: LiDAR-driver met expliciete poort en frame-ID (Output onderdrukt)
ros2 run hls_lfcd_lds_driver hlds_laser_publisher --ros-args -p port:=/dev/ttyUSB0 -p frame_id:=laser > /dev/null 2>&1 &
ym_pids+=($!)

# Node 1: UltraSonic Driver (Output onderdrukt)
ros2 run motor_controller driverUltra_node > /dev/null 2>&1 &
ym_pids+=($!)

# Node 2: Median Node (Output onderdrukt)
ros2 run motor_controller median_node > /dev/null 2>&1 &
ym_pids+=($!)

# Node 3: Lidar Avoidance (VOLLEDIGE OUTPUT IN DE TERMINAL)
env PYTHONUNBUFFERED=1 ros2 run motor_controller lidar_avoidance &
ym_pids+=($!)

# Node 4: PID Test Node (Output onderdrukt)
ros2 run motor_controller PIDtest_node > /dev/null 2>&1 &
ym_pids+=($!)

# Node 5: Motor Node (Output onderdrukt)
ros2 run motor_controller motor_node > /dev/null 2>&1 &
ym_pids+=($!)

# Node 6: Mixer Node (Output onderdrukt)
ros2 run motor_controller mixer_node > /dev/null 2>&1 &
ym_pids+=($!)

# ---------------------------------------------------------

echo "Alle nodes draaien correct via /dev/ttyUSB0. Druk op [CTRL+C] om ze allemaal veilig te stoppen."

# Keep script alive until interrupted
wait
