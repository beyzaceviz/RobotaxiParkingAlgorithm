# Teknofest Robotaksi - Autonomous Parking Algorithm

This repository contains the autonomous parking algorithm for the Teknofest Robotaksi passenger autonomous vehicle competition. It is designed to run on a ROS 2 Foxy environment natively integrated with the vehicle's hardware via CAN bus.

## Overview

The parking system utilizes a combination of LiDAR (Velodyne), Stereo Camera (ZED2), and GPS/IMU (XSENS) to safely detect parking slots, interpret traffic signs using YOLO, and execute a multi-stage parking maneuver.

### Key Features
- **Stop-and-Scan Exploration:** The vehicle stops at designated points to scan for parking signs and available slots, preventing motion blur and odometry drift during perception.
- **Dynamic Slot Selection:** Selects the nearest available parking slot dynamically based on YOLO detections mapped to GPS zones.
- **Pure-Pursuit Control:** Custom pure-pursuit steering controller with dynamic lookahead for precise maneuvering.
- **Direct CAN Actuation:** Directly interfaces with the vehicle's low-level actuators (`beemobs_actuator.py`) for steering and speed control, bypassing intermediate velocity commands for lower latency.

## Architecture & Technologies

- **Framework:** ROS 2 Foxy Fitzroy
- **Language:** Python 3.8
- **Middleware:** Fast-DDS (configured via `fastdds_udp_only.xml`)
- **Perception:** YOLOv8 (PyTorch) for traffic sign detection
- **Containerization:** Docker (Ubuntu 20.04 base)

## Repository Structure

```
.
├── docs/                   # Official Teknofest documentation and setup guides
├── scripts/                # Utility scripts (e.g., diagnostics)
├── src/                    # ROS 2 Workspace Source
│   ├── parking_safety/     # Main parking decision, mission planner, and control nodes
│   ├── robotaxi_localization/ # Odometry and EKF/UKF localization
│   ├── smart_can_msgs/     # Custom ROS 2 message definitions for CAN communication
│   └── yolo_traffic_detector/ # YOLO-based perception node
├── Dockerfile              # Docker image definition for the complete environment
├── entrypoint.sh           # Docker container entrypoint
├── fastdds_udp_only.xml    # DDS configuration for robust network communication
└── README.md               # This file
```

## Getting Started

### Prerequisites
- Docker Engine and Docker CLI
- NVIDIA Container Toolkit (for YOLO GPU acceleration)
- Access to the vehicle's network (refer to `docs/ARAC_BAGLANTI_ADIMLARI.md`)

### Building the Docker Image

Build the ROS 2 workspace into a Docker image directly on the vehicle computer or your local machine:

```bash
docker build -t park_algoritmasi_foxy .
```

### Running the Container

Run the container using host networking and GPU passthrough. The image is configured to use the provided DDS XML for reliable communication.

```bash
sudo docker run -it --rm --network host --gpus all \
    -e ROS_DOMAIN_ID=0 --name park park_algoritmasi_foxy
```

### Launching the Parking Node

Once inside the container, launch the main parking bringup script:

```bash
ros2 launch parking_safety park_bringup.launch.py
```

## Authors
Robotaksi Team (2026)
