#!/usr/bin/env python3
"""
prepare_rdd_yolo.py
===================
Converts RDD2022 India Pascal VOC XML annotations to YOLO format.
Targets Road Damage categories with special focus on D40 (Potholes).

Classes mapped:
  0: D40 (Pothole)
  1: D00 (Longitudinal Crack)
  2: D10 (Transverse Crack)
  3: D20 (Alligator Crack)
  4: D44 (Crosswalk / Line Blur)

Generates:
  - dataset/
      train/images, train/labels
      val/images, val/labels
  - rdd_dataset.yaml (Ultralytics ready config)
  - Dataset distribution statistics summary
"""

import os
import sys
import glob
import random
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

# Paths
BASE_DIR = Path(__file__).resolve().parent
XML_DIR = BASE_DIR / "train" / "annotations" / "xmls"
IMG_DIR = BASE_DIR / "train" / "images"
DATASET_OUT = BASE_DIR / "rdd_yolo_dataset"

CLASS_MAP = {
    "D40": 0,  # Pothole (Primary class)
    "D00": 1,  # Longitudinal Crack
    "D10": 2,  # Transverse Crack
    "D20": 3,  # Alligator Crack
    "D44": 4,  # Road Marking Defect
}

CLASS_NAMES = ["pothole_D40", "longitudinal_crack_D00", "transverse_crack_D10", "alligator_crack_D20", "marking_defect_D44"]


def convert_xml_to_yolo():
    print("=" * 65)
    print("  RDD2022 INDIA: VOC TO YOLO DATASET CONVERTER")
    print("=" * 65)

    if not XML_DIR.exists() or not IMG_DIR.exists():
        print(f"❌ Error: Could not find dataset directories:\n  {XML_DIR}\n  {IMG_DIR}")
        return

    xml_files = sorted(glob.glob(str(XML_DIR / "*.xml")))
    print(f"  [+] Found {len(xml_files)} XML annotation files.")

    # Create directories
    for split in ["train", "val"]:
        (DATASET_OUT / split / "images").mkdir(parents=True, exist_ok=True)
        (DATASET_OUT / split / "labels").mkdir(parents=True, exist_ok=True)

    # Shuffle and split 80% train / 20% val
    random.seed(42)
    random.shuffle(xml_files)
    split_idx = int(len(xml_files) * 0.8)
    train_xmls = xml_files[:split_idx]
    val_xmls = xml_files[split_idx:]

    stats = {name: 0 for name in CLASS_NAMES}
    total_boxes = 0
    valid_images = 0

    for split, files in [("train", train_xmls), ("val", val_xmls)]:
        print(f"  [+] Processing {split} split ({len(files)} items)...")
        for xml_path in files:
            xml_p = Path(xml_path)
            stem = xml_p.stem
            img_path = IMG_DIR / f"{stem}.jpg"
            if not img_path.exists():
                # check jpeg/png
                candidates = list(IMG_DIR.glob(f"{stem}.*"))
                if candidates:
                    img_path = candidates[0]
                else:
                    continue

            try:
                tree = ET.parse(xml_path)
                root = tree.getroot()
                size = root.find("size")
                if size is not None:
                    w = float(size.find("width").text)
                    h = float(size.find("height").text)
                else:
                    w, h = 720.0, 720.0

                label_lines = []
                for obj in root.findall("object"):
                    name = obj.find("name").text.strip()
                    if name in CLASS_MAP:
                        cls_id = CLASS_MAP[name]
                        stats[CLASS_NAMES[cls_id]] += 1
                        total_boxes += 1

                        bnd = obj.find("bndbox")
                        xmin = float(bnd.find("xmin").text)
                        ymin = float(bnd.find("ymin").text)
                        xmax = float(bnd.find("xmax").text)
                        ymax = float(bnd.find("ymax").text)

                        # Normalize
                        x_center = ((xmin + xmax) / 2.0) / w
                        y_center = ((ymin + ymax) / 2.0) / h
                        bw = (xmax - xmin) / w
                        bh = (ymax - ymin) / h

                        # Clip to bounds
                        x_center = max(0.0, min(1.0, x_center))
                        y_center = max(0.0, min(1.0, y_center))
                        bw = max(0.0, min(1.0, bw))
                        bh = max(0.0, min(1.0, bh))

                        label_lines.append(f"{cls_id} {x_center:.6f} {y_center:.6f} {bw:.6f} {bh:.6f}")

                # Copy image and write label file
                dst_img = DATASET_OUT / split / "images" / img_path.name
                shutil.copy(img_path, dst_img)

                dst_txt = DATASET_OUT / split / "labels" / f"{stem}.txt"
                with open(dst_txt, "w") as f:
                    f.write("\n".join(label_lines) + "\n")

                valid_images += 1

            except Exception as e:
                print(f"  [!] Error parsing {xml_path}: {e}")

    # Generate rdd_dataset.yaml
    yaml_content = f"""# Ultralytics YOLO RDD2022 Dataset Configuration
path: {DATASET_OUT.as_posix()}
train: train/images
val: val/images

# Classes
names:
  0: pothole_D40
  1: longitudinal_crack_D00
  2: transverse_crack_D10
  3: alligator_crack_D20
  4: marking_defect_D44
"""
    yaml_path = DATASET_OUT / "rdd_dataset.yaml"
    with open(yaml_path, "w") as f:
        f.write(yaml_content)

    print("\n" + "=" * 65)
    print("  DATASET CONVERSION COMPLETE")
    print("=" * 65)
    print(f"  Total Processed Images : {valid_images}")
    print(f"  Total Labeled Objects  : {total_boxes}")
    print("  Class Distribution:")
    for name, cnt in stats.items():
        print(f"    - {name:<26} : {cnt:>5} boxes")
    print(f"  Config YAML Saved      : {yaml_path}")
    print("=" * 65 + "\n")


if __name__ == "__main__":
    convert_xml_to_yolo()
