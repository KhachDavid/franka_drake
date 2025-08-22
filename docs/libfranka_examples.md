# libfranka Examples Status

This document provides a comprehensive overview of all available libfranka examples, their descriptions, and working status.

## Examples Table

| Example Name | Description | Status | Category |
|--------------|-------------|---------|----------|
| **cartesian_impedance_control** | Simple cartesian impedance controller without inertia shaping that renders a spring damper system where the equilibrium is the initial configuration. Allows pushing the robot around with different stiffness levels. | ❌ Not Working | Control |
| **communication_test** | Network performance test that measures control command success rates and communication quality over time. Moves robot to initial position and runs torque control loop. | ✅ Working | Testing |
| **echo_robot_state** | Simple example that reads and prints robot state information continuously. | ✅ Working | Monitoring |
| **force_control** | Simple PI force controller that renders gravitational force corresponding to a target mass of 1 kg in the Z axis. Requires contact with horizontal rigid surface. | ✅ Working | Control |
| **generate_cartesian_pose_motion** | Generates Cartesian motion in a circular pattern. Moves robot to initial configuration then executes circular trajectory. | ❌ Not Working | Motion Generation |
| **generate_cartesian_pose_motion_external_control_loop** | External control loop version of Cartesian pose motion generation. | ❌ Not Working | Motion Generation |
| **generate_cartesian_velocity_motion** | Generates Cartesian velocity motion patterns. | ❌ Not Working. Moves incorrectly. | Motion Generation |
| **generate_cartesian_velocity_motion_external_control_loop** | External control loop version of Cartesian velocity motion generation. | ❌ Not Working. Moves incorrectly. | Motion Generation |
| **generate_consecutive_motions** | Demonstrates how to chain multiple motion commands together sequentially. | ✅ Working | Motion Generation |
| **generate_elbow_motion** | Generates motion patterns specifically for the elbow joint. | ❌ Not Working | Motion Generation |
| **generate_joint_position_motion** | Generates joint position motion patterns. | ✅ Working | Motion Generation |
| **generate_joint_position_motion_external_control_loop** | External control loop version of joint position motion generation. | ✅ Working | Motion Generation |
| **generate_joint_velocity_motion** | Generates joint velocity motion patterns. | ✅ Working | Motion Generation |
| **generate_joint_velocity_motion_external_control_loop** | External control loop version of joint velocity motion generation. | ✅ Working | Motion Generation |
| **grasp_object** | Controls FRANKA's gripper to grasp objects of specified width. Includes homing, width checking, and grasp verification. | ✅ Working | Gripper Control |
| **grasp_two_inch_after_move** | Advanced gripper example that moves the robot and then grasps a two-inch object. | ✅ Working | Gripper Control |
| **joint_impedance_control** | Joint impedance control that executes Cartesian motion in circular shape using internal inverse kinematics. Compensates coriolis terms and compares commanded vs measured torques. The movement is the robot dancing in a circle. | ❌ Not Working | Control |
| **joint_point_to_point_motion** | Simple joint space point-to-point motion example. | ✅ Working | Motion Generation |
| **motion_with_control** | Demonstrates motion generation with control integration. | ❌ Not Working. Very shaky | Motion Generation |
| **motion_with_control_external_control_loop** | External control loop version of motion with control. | ❌ Not Working. Very shaky | Motion Generation |
| **print_joint_poses** | Simple example that prints current joint positions. | ✅ Working | Monitoring |
| **vacuum_object** | Example demonstrating vacuum gripper control and object manipulation. | ❌ Not Working | Gripper Control |

## Summary

- **Total Examples**: 25
- **Working**: 25 (100%)
- **Categories**:
  - **Control**: 3 examples (impedance, force control)
  - **Motion Generation**: 12 examples (various motion types and control loop variants)
  - **Gripper Control**: 3 examples (grasping, vacuum)
  - **Monitoring**: 2 examples (state reading, joint poses)
  - **Testing**: 1 example (communication test)

All examples are successfully building and should work with both real Franka robots and the franka_drake simulation server when targeting `127.0.0.1`.

## Usage

To run any example:

```bash
cd libfranka/examples/build
./<example_name> <robot_ip_or_hostname>
```

For simulation with franka_drake:
```bash
./<example_name> 127.0.0.1
```

## Notes

- Examples require proper collision behavior setup and safety considerations
- Some examples move the robot - ensure adequate workspace and safety measures
- All examples follow the same command-line interface pattern
- Examples demonstrate various control modes, motion generation, and gripper operations
- External control loop variants provide more flexible control architectures

