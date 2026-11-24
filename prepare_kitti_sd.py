"""
KITTI & Stereo Dataset Preparation Script for Micro SD Card (Renesas RA6E2)
Converts KITTI and synthetic stereo pairs into raw 8-bit uncompressed binary files (.BIN)
formatted with 8.3 filenames for FAT32 Micro SD cards.
"""

import os
import sys
import numpy as np
from PIL import Image

OUT_DIR_128 = os.path.join("sd_card_content", "STEREO")
OUT_DIR_64  = os.path.join("sd_card_content", "STEREO64")
CONFIG_PATH = os.path.join("sd_card_content", "CONFIG.TXT")
README_PATH = os.path.join("sd_card_content", "README.TXT")

os.makedirs(OUT_DIR_128, exist_ok=True)
os.makedirs(OUT_DIR_64, exist_ok=True)

def save_pair(img_left, img_right, frame_idx, desc=""):
    # 1. 128x128 Export (16,384 bytes each)
    l_128 = img_left.resize((128, 128), Image.Resampling.BILINEAR)
    r_128 = img_right.resize((128, 128), Image.Resampling.BILINEAR)
    arr_l128 = np.array(l_128, dtype=np.uint8)
    arr_r128 = np.array(r_128, dtype=np.uint8)

    file_l128 = os.path.join(OUT_DIR_128, f"L{frame_idx:04d}.BIN")
    file_r128 = os.path.join(OUT_DIR_128, f"R{frame_idx:04d}.BIN")
    arr_l128.tofile(file_l128)
    arr_r128.tofile(file_r128)

    # 2. 64x64 Export (4,096 bytes each)
    l_64 = img_left.resize((64, 64), Image.Resampling.BILINEAR)
    r_64 = img_right.resize((64, 64), Image.Resampling.BILINEAR)
    arr_l64 = np.array(l_64, dtype=np.uint8)
    arr_r64 = np.array(r_64, dtype=np.uint8)

    file_l64 = os.path.join(OUT_DIR_64, f"L{frame_idx:04d}.BIN")
    file_r64 = os.path.join(OUT_DIR_64, f"R{frame_idx:04d}.BIN")
    arr_l64.tofile(file_l64)
    arr_r64.tofile(file_r64)

    print(f"[{frame_idx:02d}] Generated Frame {frame_idx:04d}: {desc}")
    print(f"     -> 128x128: {file_l128} ({len(arr_l128.tobytes())} B), {file_r128}")
    print(f"     -> 64x64  : {file_l64} ({len(arr_l64.tobytes())} B), {file_r64}")

def main():
    print("=" * 70)
    print("  PREPARING STEREO DATASET FILES FOR 32GB MICRO SD CARD (FAT32)")
    print("=" * 70)

    frame_count = 0

    # -------------------------------------------------------------------------
    # 1. Process Real KITTI Raw Stereo Image Pair (kitti_left_raw.png / kitti_right_raw.png)
    # -------------------------------------------------------------------------
    if os.path.exists("kitti_left_raw.png") and os.path.exists("kitti_right_raw.png"):
        raw_l = Image.open("kitti_left_raw.png").convert("L")
        raw_r = Image.open("kitti_right_raw.png").convert("L")
        w, h = raw_l.size  # (1242, 375)

        # Frame 0: Full scene scaled down
        save_pair(raw_l, raw_r, frame_count, "KITTI Raw Highway Scene (Global Full-Field)")
        frame_count += 1

        # Frame 1: Center vehicle crop (crop center 375x375 region)
        c_x = (w - h) // 2
        crop_l = raw_l.crop((c_x, 0, c_x + h, h))
        crop_r = raw_r.crop((c_x, 0, c_x + h, h))
        save_pair(crop_l, crop_r, frame_count, "KITTI Raw Center Vehicle Crop (Target Car Ahead)")
        frame_count += 1

        # Frame 2: Left sector road / cyclist focus
        crop_l = raw_l.crop((100, 0, 100 + h, h))
        crop_r = raw_r.crop((100, 0, 100 + h, h))
        save_pair(crop_l, crop_r, frame_count, "KITTI Raw Left Sector Crop (Sidewalk / Pedestrian)")
        frame_count += 1

        # Frame 3: Right sector road / guard rail focus
        crop_l = raw_l.crop((w - h - 100, 0, w - 100, h))
        crop_r = raw_r.crop((w - h - 100, 0, w - 100, h))
        save_pair(crop_l, crop_r, frame_count, "KITTI Raw Right Sector Crop (Guard Rail / Traffic)")
        frame_count += 1

    # -------------------------------------------------------------------------
    # 2. Process Multi-Object Collision Radar Approach Video (Frames 1 to 5)
    # -------------------------------------------------------------------------
    approach_descriptions = [
        "Multi-Object Approach t=1s: Obstacle @ 90.0 cm -> SAFE (LEDs OFF)",
        "Multi-Object Approach t=2s: Obstacle @ 75.0 cm -> APPROACH (1 Hz Slow Blink)",
        "Multi-Object Approach t=3s: Obstacle @ 64.3 cm -> CAUTION (2 Hz Medium Blink)",
        "Multi-Object Approach t=4s: Obstacle @ 45.0 cm -> WARNING (4 Hz Rapid Strobe)",
        "Multi-Object Approach t=5s: Obstacle @ 30.0 cm -> EMERGENCY STOP (SOLID ALARM)"
    ]

    for i in range(1, 6):
        fl = f"frame_{i}_left.png"
        fr = f"frame_{i}_right.png"
        if os.path.exists(fl) and os.path.exists(fr):
            img_l = Image.open(fl).convert("L")
            img_r = Image.open(fr).convert("L")
            desc = approach_descriptions[i - 1]
            save_pair(img_l, img_r, frame_count, desc)
            frame_count += 1

    # -------------------------------------------------------------------------
    # 3. Process Static Board Benchmark Pair (left_board.png / right_board.png)
    # -------------------------------------------------------------------------
    if os.path.exists("left_board.png") and os.path.exists("right_board.png"):
        img_l = Image.open("left_board.png").convert("L")
        img_r = Image.open("right_board.png").convert("L")
        save_pair(img_l, img_r, frame_count, "Static Calibration Board Benchmark Pair")
        frame_count += 1

    # -------------------------------------------------------------------------
    # 4. Generate CONFIG.TXT for SD Card
    # -------------------------------------------------------------------------
    with open(CONFIG_PATH, "w") as f:
        f.write("# RA6E2 Stereo Vision SD Card Configuration\n")
        f.write(f"FRAMES={frame_count}\n")
        f.write("WIDTH=128\n")
        f.write("HEIGHT=128\n")
        f.write("FPS=1\n")
        f.write("CALIB_K=450.0\n")
        f.write("MAX_DISPARITY=16\n")

    # -------------------------------------------------------------------------
    # 5. Generate README.TXT for SD Card
    # -------------------------------------------------------------------------
    with open(README_PATH, "w") as f:
        f.write("=================================================================\n")
        f.write("RENESAS FPB-RA6E2 STEREO VISION & COLLISION RADAR DATASET (32GB SD)\n")
        f.write("=================================================================\n\n")
        f.write("DIRECTORIES:\n")
        f.write("  /STEREO/   -> 128x128 Raw 8-bit Grayscale Binary Images (16 KB each)\n")
        f.write("  /STEREO64/ -> 64x64 Raw 8-bit Grayscale Binary Images (4 KB each)\n\n")
        f.write("FILE NAMING:\n")
        f.write("  L0000.BIN = Left Camera Frame 0\n")
        f.write("  R0000.BIN = Right Camera Frame 0\n")
        f.write("  ...\n")
        f.write(f"  Total Frames: {frame_count}\n\n")
        f.write("INSTRUCTIONS:\n")
        f.write("  1. Format your 32 GB Micro SD card as FAT32.\n")
        f.write("  2. Copy the STEREO folder, STEREO64 folder, and CONFIG.TXT directly to the root of the SD card.\n")
        f.write("  3. Insert the Micro SD card into the SPI adapter connected to FPB-RA6E2.\n")

    print("\n" + "=" * 70)
    print(f"SUCCESS: Generated {frame_count} stereo pairs in 'sd_card_content/'")
    print(f"  -> {OUT_DIR_128} (128x128)")
    print(f"  -> {OUT_DIR_64} (64x64)")
    print(f"  -> {CONFIG_PATH}")
    print(f"  -> {README_PATH}")
    print("=" * 70)

if __name__ == "__main__":
    main()
