/*******************************************************************************************************************
 * @file    image_data.h
 * @brief   64x64 Grayscale stereo image pair stored in RA6E2 internal Code Flash.
 *          Target MCU: Renesas RA6E2 (R7FA6E2BB3CFM) - BGK-RA6E2 Board
 *******************************************************************************************************************/

#ifndef IMAGE_DATA_H_
#define IMAGE_DATA_H_

#include <stdint.h>

#define IMAGE_WIDTH          (64U)
#define IMAGE_HEIGHT         (64U)
#define IMAGE_PIXELS         (IMAGE_WIDTH * IMAGE_HEIGHT)
#define IMAGE_SIZE_BYTES     (IMAGE_PIXELS)  /* 8-bit grayscale: 4096 bytes per image */

#define IMAGE_0_CRC32        (0x4418856DU)
#define IMAGE_1_CRC32        (0x5E7940C8U)

/* Declared const to ensure allocation in Internal Code Flash (.rodata) */
extern const uint8_t g_image_0[IMAGE_SIZE_BYTES];  /* 000001_10_0.png (Left frame) */
extern const uint8_t g_image_1[IMAGE_SIZE_BYTES];  /* 000001_10_1.png (Right frame) */

#endif /* IMAGE_DATA_H_ */
