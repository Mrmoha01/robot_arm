# Roadmap

## Done

- [x] URDF/xacro model of the 4-joint arm with a 2-finger gripper
- [x] Closed-form forward and inverse kinematics with unit tests
- [x] Automatic check that the URDF matches the kinematics code
- [x] Static torque estimate against servo limits
- [x] Gazebo world, `ros2_control` configuration and launch files
- [x] IK node (`/target_point`) and a scripted pick-and-place demo

## Next: simulation polish

- [ ] Run the full demo in Gazebo, tune joint damping and controller settings, record a GIF for the README
- [ ] Straight-line (Cartesian) moves between waypoints
- [ ] Optional MoveIt 2 configuration for collision-aware planning

## Next: physical arm

Parts: 3 × MG996R, 2 × MG90S, Arduino Nano (or similar), PCA9685 servo driver, 5 – 6 V supply of at
least 5 A, plywood, screws, wire, capacitor.

- [ ] Build the structure and wire the servos
- [ ] Microcontroller firmware: receive joint angles over serial, drive the servos
- [ ] ROS 2 hardware interface or serial bridge node, selectable with a launch argument
- [ ] Calibration file (zero offsets, direction, limits per joint)
- [ ] Run the same IK node and demo on the real arm

## Later: vision

- [ ] USB camera and calibration
- [ ] OpenCV object detection
- [ ] TF2 transform from the camera frame to `base_link`
- [ ] Detect an object and pick it with the same pipeline
