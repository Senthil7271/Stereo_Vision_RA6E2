import numpy as np
from PIL import Image

WIN_RADIUS = 2
MAX_DISPARITY = 16
H, W = 128, 128

# Create rich obstacle texture (multi-scale so it resamples crisply at any distance)
np.random.seed(100)
high_res_patch = np.random.randint(40, 230, size=(64, 64), dtype=np.uint8)

frame_specs = [
    (0, 90.0,  5, 14, 48),  # t=1s: Far (small, high in horizon)
    (1, 75.0,  6, 17, 51),  # t=2s: Approaching
    (2, 60.0,  7, 21, 55),  # t=3s: Caution (mid-size)
    (3, 45.0, 10, 26, 60),  # t=4s: Warning (large)
    (4, 30.0, 15, 32, 65),  # t=5s: Critical Stop (very large, near bumper)
]

left_frames = []
right_frames = []

for f_id, target_dist, target_disp, patch_size, y_c in frame_specs:
    # 1. Background scene: Road moving forward as car drives
    road_offset = f_id * 8
    bg_l = np.full((H, W), 35, dtype=np.uint8)
    for y in range(H):
        bg_l[y, :] += int(y * 0.25)
        if y > 60:
            road_phase = (y + road_offset) % 24
            if road_phase < 12:
                bg_l[y, 63:66] = 200 # center dashed line
    bg_r = np.copy(bg_l)

    # 2. Rescale obstacle to perspective size for this distance
    patch = np.array(Image.fromarray(high_res_patch).resize((patch_size, patch_size), Image.Resampling.BILINEAR))

    # 3. Left image: obstacle centered at (x_l, y_l)
    x_l = 74 - patch_size // 2
    y_l = y_c - patch_size // 2
    im_l = np.copy(bg_l)
    im_l[y_l:y_l+patch_size, x_l:x_l+patch_size] = patch

    # 4. Right image: simultaneously moves and expands, shifted horizontally by target_disp
    x_r = x_l - target_disp
    y_r = y_l
    im_r = np.copy(bg_r)
    im_r[y_r:y_r+patch_size, x_r:x_r+patch_size] = patch

    left_frames.append(im_l)
    right_frames.append(im_r)

    # Save preview PNG for inspection
    Image.fromarray(im_l).save(f"frame_{f_id+1}_left.png")
    Image.fromarray(im_r).save(f"frame_{f_id+1}_right.png")

print("Generated 5 Left and 5 Right simultaneously moving video frames.")

# Generate image_data.h
header_content = """/*******************************************************************************************************************
 * @file    image_data.h
 * @brief   5-Frame 128x128 Stereo Video Sequence with Dynamic Simultaneous Left & Right Motion.
 *          Target MCU: Renesas RA6E2 (R7FA6E2BB3CFM) - BGK-RA6E2 Board
 *          Demonstration: 5-Second Video (1 FPS) where BOTH Left and Right cameras move simultaneously
 *                         simulating forward car motion towards an obstacle:
 *            Frame 1 (t=1s): Obstacle at 90.0 cm (Size 14 px, Disp = 5 px)  -> SAFE (LED OFF)
 *            Frame 2 (t=2s): Obstacle at 75.0 cm (Size 17 px, Disp = 6 px)  -> APPROACH (1 Hz Blink)
 *            Frame 3 (t=3s): Obstacle at 64.3 cm (Size 21 px, Disp = 7 px)  -> CAUTION (2 Hz Blink)
 *            Frame 4 (t=4s): Obstacle at 45.0 cm (Size 26 px, Disp = 10 px) -> WARNING (4 Hz Strobe)
 *            Frame 5 (t=5s): Obstacle at 30.0 cm (Size 32 px, Disp = 15 px) -> CRITICAL STOP (SOLID ON)
 *******************************************************************************************************************/

#ifndef IMAGE_DATA_H_
#define IMAGE_DATA_H_

#include <stdint.h>

#define NUM_VIDEO_FRAMES     (5U)
#define IMAGE_WIDTH          (128U)
#define IMAGE_HEIGHT         (128U)
#define IMAGE_PIXELS         (IMAGE_WIDTH * IMAGE_HEIGHT)
#define IMAGE_SIZE_BYTES     (IMAGE_PIXELS)  /* 16,384 bytes per frame */

/* 5 Dynamic Left Camera Frames in Code Flash (.rodata) */
extern const uint8_t g_video_left[NUM_VIDEO_FRAMES][IMAGE_SIZE_BYTES];

/* 5 Dynamic Right Camera Frames in Code Flash (.rodata) */
extern const uint8_t g_video_right[NUM_VIDEO_FRAMES][IMAGE_SIZE_BYTES];

/* Helper pointer arrays */
extern const uint8_t * const g_video_left_frames[NUM_VIDEO_FRAMES];
extern const uint8_t * const g_video_right_frames[NUM_VIDEO_FRAMES];

#endif /* IMAGE_DATA_H_ */
"""

with open("ra6e2_sv/src/image_data.h", "w") as f:
    f.write(header_content)
print("Wrote image_data.h")

# Generate image_data.c
with open("ra6e2_sv/src/image_data.c", "w") as f:
    f.write("/*******************************************************************************************************************\n")
    f.write(" * @file    image_data.c\n")
    f.write(" * @brief   10-Frame (5 Left + 5 Right) 128x128 Video Sequence stored in RA6E2 Internal Code Flash (.rodata).\n")
    f.write(" *          Total Flash Memory: 160 KB (0 bytes RAM).\n")
    f.write(" *******************************************************************************************************************/\n\n")
    f.write('#include "image_data.h"\n\n')

    dists = ["90.0 cm (Size 14px, d=5 px)", "75.0 cm (Size 17px, d=6 px)", "64.3 cm (Size 21px, d=7 px)", "45.0 cm (Size 26px, d=10 px)", "30.0 cm (Size 32px, d=15 px)"]

    f.write("/* 5 Dynamic Left Frames (5 x 16,384 = 81,920 bytes in Flash) */\n")
    f.write('const uint8_t g_video_left[NUM_VIDEO_FRAMES][IMAGE_SIZE_BYTES] __attribute__((used, section(".rodata"), aligned(4))) = {\n')
    for frame_idx, lf in enumerate(left_frames):
        f.write(f"    /* Left Frame {frame_idx + 1}: {dists[frame_idx]} */\n    {{\n")
        for row in range(128):
            row_bytes = lf[row, :]
            hex_str = ", ".join(f"0x{b:02X}" for b in row_bytes)
            f.write(f"        /* Row {row:03d} */ {hex_str},\n")
        f.write("    },\n")
    f.write("};\n\n")

    f.write("/* 5 Dynamic Right Frames (5 x 16,384 = 81,920 bytes in Flash) */\n")
    f.write('const uint8_t g_video_right[NUM_VIDEO_FRAMES][IMAGE_SIZE_BYTES] __attribute__((used, section(".rodata"), aligned(4))) = {\n')
    for frame_idx, rf in enumerate(right_frames):
        f.write(f"    /* Right Frame {frame_idx + 1}: {dists[frame_idx]} */\n    {{\n")
        for row in range(128):
            row_bytes = rf[row, :]
            hex_str = ", ".join(f"0x{b:02X}" for b in row_bytes)
            f.write(f"        /* Row {row:03d} */ {hex_str},\n")
        f.write("    },\n")
    f.write("};\n\n")

    f.write("const uint8_t * const g_video_left_frames[NUM_VIDEO_FRAMES] = {\n")
    f.write("    g_video_left[0], g_video_left[1], g_video_left[2], g_video_left[3], g_video_left[4]\n")
    f.write("};\n\n")

    f.write("const uint8_t * const g_video_right_frames[NUM_VIDEO_FRAMES] = {\n")
    f.write("    g_video_right[0], g_video_right[1], g_video_right[2], g_video_right[3], g_video_right[4]\n")
    f.write("};\n")

print("Successfully generated dynamic image_data.c (160 KB total in .rodata)!")

