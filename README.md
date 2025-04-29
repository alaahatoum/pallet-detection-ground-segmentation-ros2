# pallet-detection-ground-segmentation-ros2
Real-time pallet detection and ground segmentation in ROS2
This project implements a real-time pallet detection and ground segmentation system using ROS2 and deep learning. It is optimized for deployment on edge devices like the NVIDIA Jetson AGX Orin, and designed for manufacturing and warehousing environments.

Features:
- Real-time pallet detection (GMA & Euro Pallets) using YOLOv11
- Ground segmentation using semantic segmentation model
- ROS2 node publishes annotated camera images

Structure
- `training/`: Model training scripts and notebooks
- `ros2_node/`: ROS2 node implementation
- `models/`: Links or checkpoints of trained models
- `requirements.txt`: Python dependencies

