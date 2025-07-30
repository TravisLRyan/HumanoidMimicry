# G1 Robot Dual Arm Controller

This project provides a Python-based control pipeline for the **Unitree G1** humanoid robot’s dual-arm system. It leverages the Unitree SDK v2 with DDS communication for low-level joint control, real-time monitoring, and trajectory tracking using inverse kinematics from depth camera pose tracking.

## Features

- 🔧 **Joint-Space Control**: Commands joint positions and torques for both arms.
- 📡 **DDS Integration**: Uses Unitree’s DDS pub/sub architecture (`rt/lowcmd`, `rt/lowstate`) to communicate with the robot.
- 📊 **Real-Time Feedback**: Continuously reads motor positions (`q`) and velocities (`dq`).
- 🧠 **Home Positioning**: Automatically moves both arms to a neutral pose with tolerance-based verification.
- ⚙️ **Gradual Speed Ramp-Up**: Allows smooth velocity ramping for safer motion.
- 🖐️ **Support for Simulation Mode**: Optional mode for integration with simulation environments.

## File Structure

- `robot_arm_tr.py`: Main class `G1_29_ArmController` responsible for control and communication.
- `G1_29_JointIndex`: Enumerations of motor IDs for the G1 robot model.
- `DataBuffer`: Thread-safe buffer for DDS data access.
- `LowCmd_` / `LowState_`: DDS message definitions (from Unitree SDK).

## Requirements

- Python 3.8+
- Unitree SDK2 Python bindings (`unitree_sdk2py`)
- `numpy`
- `pinocchio` (for IK, optional depending on usage)
- Properly configured DDS environment (e.g., CycloneDDS)

