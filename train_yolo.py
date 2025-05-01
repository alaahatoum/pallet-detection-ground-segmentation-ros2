import os
import ultralytics
from ultralytics import YOLO

# -----------------------------
# CONFIGURATION
# -----------------------------
MODEL_TYPE = "yolov8n.pt"  # use yolov8s.pt or yolov8m.pt if you have a stronger machine
DATA_YAML = "/home/alaa/Desktop/pallet-detection-ground-segmentation-ros2/industry_dataset3-1/data.yaml"    # must point to your dataset YAML
EPOCHS = 50
IMAGE_SIZE = 640
BATCH_SIZE = 16

PROJECT_DIR = "outputs"
RUN_NAME = "pallet_detector"

# -----------------------------
# TRAINING
# -----------------------------
def train():
    print(f"[INFO] Starting YOLOv8 training on {DATA_YAML}...")
    
    model = YOLO(MODEL_TYPE)
    
    results = model.train(
        data=DATA_YAML,
        epochs=EPOCHS,
        imgsz=IMAGE_SIZE,
        batch=BATCH_SIZE,
        project=PROJECT_DIR,
        name=RUN_NAME,
        verbose=True
    )
    
    print(f"\n[✓] Training complete!")
    print(f"[📂] Weights saved to: {results.save_dir}/weights/best.pt")
    print(f"[📊] Training curves saved to: {results.save_dir}/results.png")

if __name__ == "__main__":
    train()
