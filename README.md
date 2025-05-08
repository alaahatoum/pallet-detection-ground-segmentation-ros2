
# Pallet Detection & Ground Segmentation (ROS 2)

This project implements a real-time pallet detection and ground segmentation system using ROS 2 and deep learning. It is optimized for deployment on edge devices such as the NVIDIA Jetson AGX Orin, and is designed for use in manufacturing and warehouse environments.

## 🚀 Features

- **Pallet Detection**: Detects Euro and GMA pallets using YOLOv8.
- **Ground Segmentation**: Performs semantic segmentation to identify ground areas.
- **ROS 2 Integration**: Publishes annotated camera images through a ROS 2 node.

## 📁 Project Structure

```
pallet-detection-ground-segmentation-ros2/
├── models/                   # Pretrained YOLOv8 and segmentation models
├── pallet_ground_package/    # ROS 2 package
│   ├── pallet_ground_package/
│   │   ├── __init__.py
│   │   └── pallet_ground_node.py
│   ├── setup.py
│   └── package.xml
├── scripts/
│   └── test_image_inference.py  # Standalone image testing script
├── requirements.txt
└── README.md
```

## ⚙️ Setup Instructions

### 1. Prerequisites

- **Hardware**: NVIDIA Jetson AGX Orin with ZED 2i camera.
- **Operating System**: Ubuntu 22.04.
- **ROS 2**: Humble Hawksbill.
- **Python**: 3.10.

### 2. Clone the Repository

```bash
git clone https://github.com/alaahatoum/pallet-detection-ground-segmentation-ros2.git
cd pallet-detection-ground-segmentation-ros2
```

### 3. Install Dependencies

It's recommended to use a virtual environment:

```bash
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

### 4. Build the ROS 2 Package

```bash
cd src
colcon build
source install/setup.bash
```

## 🧪 Testing the System

### A. Live Camera Feed (ZED 2i)

To run the ROS 2 node with live camera input:

```bash
ros2 run pallet_ground_package pallet_ground_node
```

Ensure that the ZED 2i camera is publishing images to the expected topic. If necessary, adjust the topic names in the `pallet_ground_node.py` script or use ROS 2 remapping.

### B. Static Image Inference

For testing with a static image:

```bash
python3 scripts/test_image_inference.py --image_path path/to/your/image.jpg
```

Replace `path/to/your/image.jpg` with the actual path to your test image.

## 🧠 Models

The `models/` directory contains the pretrained models:

- `pallet_det.pt`: YOLOv8 model for pallet detection.
- `ground_seg.pt`: Semantic segmentation model for ground detection.

Note: There is no direct link to the models. However, the code used to train the models is located in the `train_yolo_*` folders. These folders contain the scripts and configuration used for training the models on Google Colab.

## ⚠️ Known Issues

- **NumPy Compatibility**: Some modules may not be compatible with NumPy 2.x. If you encounter errors related to NumPy versions, consider downgrading to NumPy 1.x:

  ```bash
  pip install numpy==1.24.0
  ```

- **OpenCV Errors**: If you receive errors related to `cv2.imshow`, ensure that the image path is correct and the image file exists.

## 📌 Notes

Due to limited GPU access while running the model training on Google Colab, the ground segmentation model was not trained extensively. Future improvements could involve more comprehensive training to enhance segmentation accuracy under varying conditions.


For any questions or issues, please open an issue on the [GitHub repository](https://github.com/alaahatoum/pallet-detection-ground-segmentation-ros2/issues).