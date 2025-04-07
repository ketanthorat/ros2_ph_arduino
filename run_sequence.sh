#!/bin/bash

# Source ROS workspace
source /opt/ros/humble/setup.bash
source ~/abc/ros2_ph_arduino/install/setup.bash

# Run the servo control node in the background
echo "Starting servo control node..."
$HOME/abc/ros2_ph_arduino/install/arduino_servo_control/bin/servo_control &
SERVO_PID=$!

# Give servo node time to start
sleep 1

# Run the sequence publisher
echo "Starting sequence publisher..."
$HOME/abc/ros2_ph_arduino/install/arduino_servo_control/bin/sequence_publisher

# Kill the servo node after the sequence publisher finishes
kill $SERVO_PID
echo "All done!" 