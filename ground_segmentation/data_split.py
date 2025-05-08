import os
import shutil
import random

# ==== CONFIG ====
SRC_IMAGES = "ground_segmentation/augmented/data"
SRC_LABELS = "ground_segmentation/augmented/labels"
DST_BASE = "seg_dataset"
SPLITS = [0.7, 0.2, 0.1]  # train, val, test

# ==== SETUP DESTINATION FOLDERS ====
for split in ['train', 'val', 'test']:
    os.makedirs(os.path.join(DST_BASE, 'images', split), exist_ok=True)
    os.makedirs(os.path.join(DST_BASE, 'labels', split), exist_ok=True)

# ==== COLLECT FILES ====
image_files = [f for f in os.listdir(SRC_IMAGES) if f.endswith((".jpg", ".jpeg", ".png"))]
random.shuffle(image_files)

n_total = len(image_files)
n_train = int(n_total * SPLITS[0])
n_val = int(n_total * SPLITS[1])

train_files = image_files[:n_train]
val_files = image_files[n_train:n_train + n_val]
test_files = image_files[n_train + n_val:]

def copy_files(files, split):
    for img_file in files:
        label_file = os.path.splitext(img_file)[0] + ".txt"
        shutil.copy(os.path.join(SRC_IMAGES, img_file), os.path.join(DST_BASE, 'images', split, img_file))
        shutil.copy(os.path.join(SRC_LABELS, label_file), os.path.join(DST_BASE, 'labels', split, label_file))

copy_files(train_files, 'train')
copy_files(val_files, 'val')
copy_files(test_files, 'test')

print(f" Dataset split complete: {len(train_files)} train, {len(val_files)} val, {len(test_files)} test.")
