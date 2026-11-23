/*******************************************************************************************************************
 * @file    image_data.h
 * @brief   Complex Multi-Object 128x128 Stereo Video Sequence (5 Frames @ 1 FPS).
 *          Target MCU: Renesas RA6E2 (Arm Cortex-M33 @ 200 MHz, 40 KB SRAM, 256 KB Flash)
 *
 *          Scene Configuration:
 *            - Object 1 (Left Zone):   Pedestrian on sidewalk (Distance ~90 cm, Disparity = 5 px)
 *            - Object 2 (Center Zone): Approaching Vehicle (Distance: 90 -> 75 -> 64 -> 45 -> 30 cm)
 *            - Object 3 (Right Zone):  Traffic Signpost on shoulder (Distance ~75 cm, Disparity = 6 px)
 *            - Environment:            Asphalt roadway with perspective lane markings, horizon trees, sky
 *
 *          Memory Architecture:
 *            Stored in Code Flash (.rodata): 160 KB total (0 bytes RAM).
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
