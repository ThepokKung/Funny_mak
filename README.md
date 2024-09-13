# Overview

ROS 2 Motor Control and Target Scheduling Project

This ROS 2 package provides a motor control system with random target generation, PID-based motor control, and communication between nodes using a heartbeat protocol. The package includes four primary nodes:
    controller_node: Implements PID control for motor positioning and communicates with other nodes to control motor movement.
    encoder_node: Reads motor speed and position and publishes it for use by the controller.
    scheduler_node: Schedules and sends target positions to the controller.
    random_target_node: Generates random target positions and saves them to a YAML file for the scheduler to use.

Features

PID Control: The controller_node uses a PID controller to manage the motor position.
    Heartbeat Protocol: Nodes communicate with each other using a simple heartbeat mechanism to ensure readiness and synchronization.
    Random Target Generation: The random_target_node generates random targets and saves them in a YAML file.
    YAML Configuration: Target positions are read and written to YAML files for easy configuration.
## Structure


```bash
.
├── src/
│   ├── controller_node.py           # Controls motor position with PID
│   ├── encoder_node.py              # Publishes motor speed and position
│   ├── scheduler_node.py            # Sends target positions to the controller
│   └── random_target_node.py        # Generates random targets and saves them to a YAML file
├── config/
│   └── via_point.yaml               # YAML file to store generated target positions
├── README.md                        # Project documentation
├── CMakeLists.txt                   # ROS 2 package configuration
├── package.xml                      # ROS 2 package information

```

## Installation

Install my workspace

```bash
git clone https://github.com/ThepokKung/Funny_mak.git
cd Funny_mak
```

Build Project

```bash
colcon build --parallel-workers 2 #ROS 
source install/setup.bash
 ```

## Usage/Examples

```bash
ros2 launch funny_mak fun35.launch.py 
```


## Parameters

controller_node:

    Kp, Ki, Kd: PID control parameters. These can be dynamically updated via the SetKParam service.

random_target_node:

    num_targets: Number of random targets to generate.
    target_min: Minimum value for random targets.
    target_max: Maximum value for random targets.
    file_yaml_path: The path where the random target YAML file will be saved.
## Services
controller_node provides the SetKParam service to dynamically set the PID parameters. You can call this service to update the PID parameters (Kp, Ki, Kd) in real time.

how to use service co
```bash
ros2 service call /controller_node/k_param fun35_controller_interfaces/srv/SetKParam "{kp: 0.1, ki: 0.01, kd: 0.0}"
```
