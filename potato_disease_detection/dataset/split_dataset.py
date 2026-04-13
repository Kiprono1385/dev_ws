import os
import shutil
import random
from pathlib import Path

source_dir = Path("~/Downloads/archive/PlantVillage/Potato___Late_blight").expanduser()
train_dir  = Path("/home/sierra-95/Documents/potato_disease_detection/dataset/train/Late_blight").expanduser()
val_dir    = Path("/home/sierra-95/Documents/potato_disease_detection/dataset/val/Late_blight").expanduser()

split_ratio = 0.8   # 80% train, 20% val

# Create destination folders if they don't exist
train_dir.mkdir(parents=True, exist_ok=True)
val_dir.mkdir(parents=True, exist_ok=True)

# Collect all image file paths
images = [f for f in source_dir.iterdir() if f.is_file()]
random.shuffle(images) 

# Split
split_point = int(len(images) * split_ratio)
train_files = images[:split_point]
val_files   = images[split_point:]

print(f"Total images: {len(images)}")
print(f"Train: {len(train_files)}, Val: {len(val_files)}")

# Copy files
for f in train_files:
    shutil.copy2(f, train_dir / f.name)

for f in val_files:
    shutil.copy2(f, val_dir / f.name)

print("Split complete.")
