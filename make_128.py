import numpy as np
from PIL import Image

# 1. Load 64x64 images and upscale to 128x128
im_l = Image.open('left_board.png').resize((128, 128), Image.Resampling.BICUBIC)
im_r = Image.open('right_board.png').resize((128, 128), Image.Resampling.BICUBIC)

im_l.save('left_128.png')
im_r.save('right_128.png')

arr_l = np.array(im_l, dtype=np.uint8)
arr_r = np.array(im_r, dtype=np.uint8)

print(f"Generated 128x128 images: Left {arr_l.shape}, Right {arr_r.shape}")

# 2. Write image_data.h
header_code = """/*******************************************************************************************************************
 * @file    image_data.h
 * @brief   128x128 Grayscale stereo image pair stored in RA6E2 internal Code Flash.
 *          Target MCU: Renesas RA6E2 (R7FA6E2BB3CFM) - BGK-RA6E2 Board
 *******************************************************************************************************************/

#ifndef IMAGE_DATA_H_
#define IMAGE_DATA_H_

#include <stdint.h>

#define IMAGE_WIDTH          (128U)
#define IMAGE_HEIGHT         (128U)
#define IMAGE_PIXELS         (IMAGE_WIDTH * IMAGE_HEIGHT)
#define IMAGE_SIZE_BYTES     (IMAGE_PIXELS)  /* 8-bit grayscale: 16384 bytes per image */

/* Declared const to ensure allocation in Internal Code Flash (.rodata) */
extern const uint8_t g_image_0[IMAGE_SIZE_BYTES];  /* Left frame (128x128) */
extern const uint8_t g_image_1[IMAGE_SIZE_BYTES];  /* Right frame (128x128) */

#endif /* IMAGE_DATA_H_ */
"""

with open("ra6e2_sv/src/image_data.h", "w") as f:
    f.write(header_code)

# 3. Write image_data.c
with open("ra6e2_sv/src/image_data.c", "w") as f:
    f.write("/*******************************************************************************************************************\n")
    f.write(" * @file    image_data.c\n")
    f.write(" * @brief   128x128 Image byte definitions for RA6E2 Internal Code Flash storage (.rodata).\n")
    f.write(" *******************************************************************************************************************/\n\n")
    f.write('#include "image_data.h"\n\n')

    f.write("/* Image 0: Left frame (128x128 Grayscale, 16384 bytes in Flash) */\n")
    f.write('const uint8_t g_image_0[IMAGE_SIZE_BYTES] __attribute__((used, section(".rodata"), aligned(4))) = {\n')
    for row in range(128):
        row_bytes = arr_l[row, :]
        hex_str = ", ".join(f"0x{b:02X}" for b in row_bytes)
        f.write(f"    /* Row {row:03d} */ {hex_str},\n")
    f.write("};\n\n")

    f.write("/* Image 1: Right frame (128x128 Grayscale, 16384 bytes in Flash) */\n")
    f.write('const uint8_t g_image_1[IMAGE_SIZE_BYTES] __attribute__((used, section(".rodata"), aligned(4))) = {\n')
    for row in range(128):
        row_bytes = arr_r[row, :]
        hex_str = ", ".join(f"0x{b:02X}" for b in row_bytes)
        f.write(f"    /* Row {row:03d} */ {hex_str},\n")
    f.write("};\n")

print("Successfully generated 128x128 image_data.h and image_data.c!")

