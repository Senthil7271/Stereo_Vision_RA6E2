import numpy as np

np.random.seed(42)
WIN_RADIUS = 2
MAX_DISPARITY = 16
H, W = 128, 128

# Base background: smooth gradient with low-frequency floor
bg = np.full((H, W), 40, dtype=np.uint8)
for y in range(H):
    bg[y, :] += int(y * 0.3)

PATCH_SIZE = 24
patch = np.random.randint(30, 240, size=(PATCH_SIZE, PATCH_SIZE), dtype=np.uint8)

x_l = 72
y_l = 50

im_l = np.copy(bg)
im_l[y_l:y_l+PATCH_SIZE, x_l:x_l+PATCH_SIZE] = patch

# Target shifts for [90cm, 75cm, 60cm, 45cm, 30cm]:
shifts = [5, 6, 7, 10, 15]

right_frames = []
for shift in shifts:
    im_r = np.copy(bg)
    x_r = x_l - shift
    im_r[y_l:y_l+PATCH_SIZE, x_r:x_r+PATCH_SIZE] = patch
    right_frames.append(im_r)

print("Generated 1 Left frame and 5 Right frames.")

# Generate image_data.h
header_content = """/*******************************************************************************************************************
 * @file    image_data.h
 * @brief   5-Frame 128x128 Stereo Video Sequence stored in Internal Code Flash (.rodata).
 *          Target MCU: Renesas RA6E2 (R7FA6E2BB3CFM) - BGK-RA6E2 Board
 *          Demonstration: 5-Second Video (1 FPS) simulating an obstacle approaching:
 *            Frame 1 (t=1s): Obstacle at 90.0 cm (Disparity = 5 px)  -> SAFE (LED OFF)
 *            Frame 2 (t=2s): Obstacle at 75.0 cm (Disparity = 6 px)  -> APPROACH (1 Hz Blink)
 *            Frame 3 (t=3s): Obstacle at 64.3 cm (Disparity = 7 px)  -> CAUTION (2 Hz Blink)
 *            Frame 4 (t=4s): Obstacle at 45.0 cm (Disparity = 10 px) -> WARNING (4 Hz Strobe)
 *            Frame 5 (t=5s): Obstacle at 30.0 cm (Disparity = 15 px) -> CRITICAL STOP (SOLID ON)
 *******************************************************************************************************************/

#ifndef IMAGE_DATA_H_
#define IMAGE_DATA_H_

#include <stdint.h>

#define NUM_VIDEO_FRAMES     (5U)
#define IMAGE_WIDTH          (128U)
#define IMAGE_HEIGHT         (128U)
#define IMAGE_PIXELS         (IMAGE_WIDTH * IMAGE_HEIGHT)
#define IMAGE_SIZE_BYTES     (IMAGE_PIXELS)  /* 16,384 bytes per frame */

/* Common Reference Left Frame in Code Flash (.rodata) */
extern const uint8_t g_video_left[IMAGE_SIZE_BYTES];

/* 5 Right Camera Frames in Code Flash (.rodata) corresponding to approaching distance */
extern const uint8_t g_video_right[NUM_VIDEO_FRAMES][IMAGE_SIZE_BYTES];

/* Helper pointer array */
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
    f.write(" * @brief   5-Frame 128x128 Video Sequence stored in RA6E2 Internal Code Flash (.rodata).\n")
    f.write(" *******************************************************************************************************************/\n\n")
    f.write('#include "image_data.h"\n\n')

    f.write("/* Common Left Reference Frame (16,384 bytes in Flash) */\n")
    f.write('const uint8_t g_video_left[IMAGE_SIZE_BYTES] __attribute__((used, section(".rodata"), aligned(4))) = {\n')
    for row in range(128):
        row_bytes = im_l[row, :]
        hex_str = ", ".join(f"0x{b:02X}" for b in row_bytes)
        f.write(f"    /* Row {row:03d} */ {hex_str},\n")
    f.write("};\n\n")

    f.write("/* 5 Right Frames (5 x 16,384 bytes = 81,920 bytes in Flash) */\n")
    f.write('const uint8_t g_video_right[NUM_VIDEO_FRAMES][IMAGE_SIZE_BYTES] __attribute__((used, section(".rodata"), aligned(4))) = {\n')
    dists = ["90.0 cm (d=5 px)", "75.0 cm (d=6 px)", "64.3 cm (d=7 px)", "45.0 cm (d=10 px)", "30.0 cm (d=15 px)"]
    for frame_idx, rf in enumerate(right_frames):
        f.write(f"    /* Frame {frame_idx + 1}: Target Distance {dists[frame_idx]} */\n    {{\n")
        for row in range(128):
            row_bytes = rf[row, :]
            hex_str = ", ".join(f"0x{b:02X}" for b in row_bytes)
            f.write(f"        /* Row {row:03d} */ {hex_str},\n")
        f.write("    },\n")
    f.write("};\n\n")

    f.write("const uint8_t * const g_video_right_frames[NUM_VIDEO_FRAMES] = {\n")
    f.write("    g_video_right[0],\n")
    f.write("    g_video_right[1],\n")
    f.write("    g_video_right[2],\n")
    f.write("    g_video_right[3],\n")
    f.write("    g_video_right[4]\n")
    f.write("};\n")

print("Successfully generated image_data.c (96 KB total in .rodata)!")

