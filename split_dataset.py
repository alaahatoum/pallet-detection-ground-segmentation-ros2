import os
import random
import shutil
from glob import glob

# === CONFIG ===
AUGMENTED_DIR = "augmented_dataset"         # Folder containing augmented images and labels
SPLIT_DIR = "splitted_dataset"               # Folder where the split datasets will be saved
TRAIN_SPLIT = 0.7                          # 80% for training
VAL_SPLIT = 0.2                              # 10% for validation
TEST_SPLIT = 0.1                             # 10% for testing

# === Create output directories ===
os.makedirs(os.path.join(SPLIT_DIR, "train/images"), exist_ok=True)
os.makedirs(os.path.join(SPLIT_DIR, "train/labels"), exist_ok=True)
os.makedirs(os.path.join(SPLIT_DIR, "val/images"), exist_ok=True)
os.makedirs(os.path.join(SPLIT_DIR, "val/labels"), exist_ok=True)
os.makedirs(os.path.join(SPLIT_DIR, "test/images"), exist_ok=True)
os.makedirs(os.path.join(SPLIT_DIR, "test/labels"), exist_ok=True)

# === Get all image paths (with corresponding txt labels) ===
image_paths = glob(os.path.join(AUGMENTED_DIR, "*.jpg"))
label_paths = [path.replace(".jpg", ".txt") for path in image_paths]

# === Shuffle the dataset ===
combined = list(zip(image_paths, label_paths))
random.shuffle(combined)
image_paths, label_paths = zip(*combined)

# === Split dataset ===
total_count = len(image_paths)
train_count = int(total_count * TRAIN_SPLIT)
val_count = int(total_count * VAL_SPLIT)
test_count = total_count - train_count - val_count

# Train, Validation, Test splits
train_images = image_paths[:train_count]
train_labels = label_paths[:train_count]

val_images = image_paths[train_count:train_count + val_count]
val_labels = label_paths[train_count:train_count + val_count]

test_images = image_paths[train_count + val_count:]
test_labels = label_paths[train_count + val_count:]

# === Move files to corresponding directories ===
def move_files(image_paths, label_paths, set_type):
    for img_path, lbl_path in zip(image_paths, label_paths):
        img_filename = os.path.basename(img_path)
        lbl_filename = os.path.basename(lbl_path)

        # Move image and label
        shutil.move(img_path, os.path.join(SPLIT_DIR, set_type, "images", img_filename))
        shutil.move(lbl_path, os.path.join(SPLIT_DIR, set_type, "labels", lbl_filename))

# Move images and labels for each split
move_files(train_images, train_labels, "train")
move_files(val_images, val_labels, "val")
move_files(test_images, test_labels, "test")

print(f"Dataset split complete!")
