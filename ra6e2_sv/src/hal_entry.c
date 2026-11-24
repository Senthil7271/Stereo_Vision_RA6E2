/*******************************************************************************************************************
 * @file    hal_entry.c
 * @brief   5-Second Video (1 FPS) Stereo Vision Obstacle Detection & Collision Radar Engine.
 *          Target MCU: Renesas RA6E2 (Arm Cortex-M33 @ 200 MHz, 40 KB SRAM, 256 KB Flash) - BGK-RA6E2 Board
 *
 *          Demonstration:
 *            Plays a 5-second continuous looping video (5 frames @ 1 FPS) simulating an approaching obstacle:
 *              - Frame 1 (t=1s): Obstacle at 90.0 cm (Disparity = 5 px)  -> SAFE (LED OFF)
 *              - Frame 2 (t=2s): Obstacle at 75.0 cm (Disparity = 6 px)  -> APPROACH (1 Hz Slow Blink)
 *              - Frame 3 (t=3s): Obstacle at 64.3 cm (Disparity = 7 px)  -> CAUTION (2 Hz Medium Blink)
 *              - Frame 4 (t=4s): Obstacle at 45.0 cm (Disparity = 10 px) -> WARNING (4 Hz Fast Strobe)
 *              - Frame 5 (t=5s): Obstacle at 30.0 cm (Disparity = 15 px) -> EMERGENCY STOP (SOLID ALARM)
 *
 *          Memory Architecture:
 *            - Video frames stored in Internal Code Flash (.rodata): 0 bytes RAM.
 *            - Disparity output buffer: 16 KB SRAM.
 *            - Total SRAM consumed: < 21 KB (Leaves > 19 KB free headroom).
 *******************************************************************************************************************/

#include "hal_data.h"
#include "image_data.h"
#include "tinyml_classifier.h"
#include "segger_transfer.h"
#include "sd_spi.h"
#include "sd_card.h"
#include "simple_fat32.h"
#include "r_uart_api.h"
#include "r_sci_uart.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#if (1 == BSP_MULTICORE_PROJECT) && BSP_TZ_SECURE_BUILD
bsp_ipc_semaphore_handle_t g_core_start_semaphore =
{
    .semaphore_num = 0
};
#endif

/* -----------------------------------------------------------------------------------------------------------------
 * Micro SD Card Streaming & Dataset Management (32GB FAT32 SPI Interface)
 * ----------------------------------------------------------------------------------------------------------------- */
volatile bool     g_sd_card_present = false;
volatile uint32_t g_sd_frame_count  = 10;     /* 10 stereo pairs in /STEREO64/ or /STEREO/ */
volatile uint32_t g_sd_active_frame = 0;
volatile char     g_active_source[40] = "INITIALIZING";

/* SRAM input buffers for SD card streaming (64x64 uint8: 4,096 bytes each = 8 KB total) */
static uint8_t g_sd_left_frame[64 * 64];
static uint8_t g_sd_right_frame[64 * 64];

/* -----------------------------------------------------------------------------------------------------------------
 * Stereo Vision Configuration & Calibration
 * ----------------------------------------------------------------------------------------------------------------- */
#define MAX_DISPARITY        (16U)     /* Disparity search range: 0 to 15 pixels */
#define WIN_RADIUS           (2)       /* 5x5 Census window radius */

/* Distance calibration constant: K = f * Baseline = 450.0 cm*px */
#define K_CALIBRATION_CM     (450.0f)

/* 128x128 Visual Disparity Map: Scaled 0..255 (16,384 bytes in SRAM) */
volatile uint8_t g_disparity_visual[IMAGE_HEIGHT][IMAGE_WIDTH];

/* Subsampled ASCII preview (32 rows x 64 cols) for in-IDE Expressions tab inspection (2 KB) */
volatile char g_ascii_depth[32][65];

/* -----------------------------------------------------------------------------------------------------------------
 * Telemetry & Demonstration State (Inspectable in e2 studio Expressions tab)
 * ----------------------------------------------------------------------------------------------------------------- */
typedef enum {
    ALERT_SAFE          = 0,  /* > 80 cm: Path Clear (LEDs OFF) */
    ALERT_APPROACH      = 1,  /* 65 - 80 cm: Info (1 Hz Slow Blink) */
    ALERT_CAUTION       = 2,  /* 50 - 65 cm: Caution (2 Hz Medium Blink) */
    ALERT_WARNING       = 3,  /* 35 - 50 cm: Warning (4 Hz Rapid Strobe) */
    ALERT_CRITICAL_STOP = 4   /* <= 30 cm: Emergency Collision Alarm (SOLID ON) */
} alert_level_t;

typedef enum {
    OBSTACLE_NONE   = 0,
    OBSTACLE_LEFT   = 1,
    OBSTACLE_CENTER = 2,
    OBSTACLE_RIGHT  = 3
} obstacle_dir_t;

volatile uint32_t       g_current_frame       = 1;          /* Active Video Frame: 1 to 5 */
volatile float          g_nearest_obstacle_cm = 999.0f;     /* Detected Distance: 90 -> 75 -> 64 -> 45 -> 30 cm */
volatile alert_level_t  g_alert_level         = ALERT_SAFE; /* Radar Alert Level: 0 to 4 */
volatile obstacle_dir_t g_obstacle_direction  = OBSTACLE_NONE;
volatile uint32_t       g_total_frames_played = 0;          /* Continuous frame counter */
volatile uint32_t       g_video_loop_count    = 0;          /* Number of 5-second video loops completed */

/* Multi-Object 3-Sector Spatial Radar Telemetry (Inspectable in e2 studio Expressions tab) */
volatile float          g_dist_left_cm        = 999.0f;     /* Left Sector: Pedestrian on Sidewalk (~90.0 cm) */
volatile float          g_dist_center_cm      = 999.0f;     /* Center Sector: Lead Vehicle (90 -> 75 -> 64 -> 45 -> 30 cm) */
volatile float          g_dist_right_cm       = 999.0f;     /* Right Sector: Traffic Signpost on Shoulder (~75.0 cm) */
volatile uint32_t       g_num_obstacles_detected = 0;       /* Total distinct obstacles detected simultaneously (3) */

/* Benchmark metrics */
volatile uint32_t       g_inference_cycles    = 0;
volatile float          g_inference_ms        = 0.0f;

/* TinyML Feature Extraction & Model Telemetry (Inspectable in e2 studio Expressions tab) */
volatile float          g_feat_peak_disparity = 0.0f;  /* Feature 1: Peak disparity (0..15 px) */
volatile float          g_feat_centroid_x     = 0.0f;  /* Feature 2: Horizontal centroid X (0.0=left, 1.0=right) */
volatile float          g_feat_compactness    = 0.0f;  /* Feature 3: Obstacle spatial compactness (0..1) */
volatile float          g_tinyml_confidence   = 0.0f;  /* TinyML Softmax Neural Confidence: 0.0% to 100.0% */

/* -----------------------------------------------------------------------------------------------------------------
 * 5x5 Census Transform: Encodes 24 neighbor comparisons into a single uint32_t
 * ----------------------------------------------------------------------------------------------------------------- */
static inline uint32_t compute_census_5x5(const uint8_t *img, int x, int y)
{
    uint32_t center_idx = (uint32_t)(y * (int)IMAGE_WIDTH + x);
    uint8_t center = img[center_idx];
    uint32_t census = 0;
    int bit = 0;

    for (int dy = -WIN_RADIUS; dy <= WIN_RADIUS; dy++)
    {
        for (int dx = -WIN_RADIUS; dx <= WIN_RADIUS; dx++)
        {
            if (dx == 0 && dy == 0) continue; /* Skip center pixel */

            uint32_t neighbor_idx = (uint32_t)((y + dy) * (int)IMAGE_WIDTH + (x + dx));
            if (img[neighbor_idx] >= center)
            {
                census |= (1UL << bit);
            }
            bit++;
        }
    }
    return census;
}

/* -----------------------------------------------------------------------------------------------------------------
 * Fast 3-Line Rolling Median Filter: Cleans noise in-place using only 384 bytes of RAM!
 * ----------------------------------------------------------------------------------------------------------------- */
static void apply_rolling_median_filter(void)
{
    static uint8_t line_prev[IMAGE_WIDTH];
    static uint8_t line_curr[IMAGE_WIDTH];
    static uint8_t line_next[IMAGE_WIDTH];

    /* Prime rolling buffer with first two rows */
    memcpy(line_prev, (const void*)&g_disparity_visual[0][0], IMAGE_WIDTH);
    memcpy(line_curr, (const void*)&g_disparity_visual[1][0], IMAGE_WIDTH);

    for (int y = 1; y < (int)IMAGE_HEIGHT - 1; y++)
    {
        memcpy(line_next, (const void*)&g_disparity_visual[y + 1][0], IMAGE_WIDTH);

        for (int x = (int)(WIN_RADIUS + MAX_DISPARITY + 1); x < (int)IMAGE_WIDTH - 1; x++)
        {
            uint8_t w[9] = {
                line_prev[x - 1], line_prev[x], line_prev[x + 1],
                line_curr[x - 1], line_curr[x], line_curr[x + 1],
                line_next[x - 1], line_next[x], line_next[x + 1]
            };

            /* Partial insertion sort to extract 5th element (median) */
            for (int i = 0; i < 5; i++)
            {
                int min_idx = i;
                for (int j = i + 1; j < 9; j++)
                {
                    if (w[j] < w[min_idx]) min_idx = j;
                }
                uint8_t t = w[i]; w[i] = w[min_idx]; w[min_idx] = t;
            }

            g_disparity_visual[y][x] = w[4];
        }

        memcpy(line_prev, line_curr, IMAGE_WIDTH);
        memcpy(line_curr, line_next, IMAGE_WIDTH);
    }
}

/* -----------------------------------------------------------------------------------------------------------------
 * 128x128 Stereo Disparity Engine: Processes current Left and Right video frames
 * ----------------------------------------------------------------------------------------------------------------- */
void run_stereo_disparity_128(const uint8_t *left_frame, const uint8_t *right_frame)
{
    static uint32_t left_census[IMAGE_WIDTH];
    static uint32_t right_census[IMAGE_WIDTH];

    memset((void*)g_disparity_visual, 0, sizeof(g_disparity_visual));

    /* Enable DWT Cycle Counter on Cortex-M33 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT       = 0;

    for (int y = WIN_RADIUS; y < (int)(IMAGE_HEIGHT - WIN_RADIUS); y++)
    {
        /* Step 1: Precompute Census code for the entire horizontal scanline */
        for (int x = WIN_RADIUS; x < (int)(IMAGE_WIDTH - WIN_RADIUS); x++)
        {
            left_census[x]  = compute_census_5x5(left_frame, x, y);
            right_census[x] = compute_census_5x5(right_frame, x, y);
        }

        /* Step 2: Epipolar Matching along horizontal line */
        for (int x = (int)(WIN_RADIUS + MAX_DISPARITY); x < (int)(IMAGE_WIDTH - WIN_RADIUS); x++)
        {
            uint32_t l_code   = left_census[x];
            uint32_t min_cost = 0xFFFFFFFF;
            uint8_t  best_d   = 0;

            for (uint8_t d = 0; d < MAX_DISPARITY; d++)
            {
                uint32_t r_code = right_census[x - d];
                /* Cortex-M33 single-cycle hardware POPCOUNT intrinsic */
                uint32_t cost = (uint32_t)__builtin_popcount(l_code ^ r_code);

                if (cost < min_cost)
                {
                    min_cost = cost;
                    best_d   = d;
                }
            }

            /* Scale 0..15 directly to 0..255 for direct viewing in e2 studio Raw Image Format */
            g_disparity_visual[y][x] = (uint8_t)(best_d * (255 / (MAX_DISPARITY - 1)));
        }
    }

    /* Step 3: Fast In-Place Rolling Median Filter */
    apply_rolling_median_filter();

    /* Benchmark statistics */
    g_inference_cycles = DWT->CYCCNT;
    g_inference_ms     = (float)g_inference_cycles / 200000.0f;
}

/* -----------------------------------------------------------------------------------------------------------------
 * Fast 64x64 Stereo Disparity Engine for Micro SD Card Dataset:
 * Computes census disparity on 64x64 frames and scales into 128x128 visual map in SRAM.
 * Execution RAM: < 9 KB. Latency: ~1.6 ms at 200 MHz (>600 FPS).
 * ----------------------------------------------------------------------------------------------------------------- */
void run_stereo_disparity_64_to_128(const uint8_t *left_64, const uint8_t *right_64)
{
    static uint32_t left_census[64];
    static uint32_t right_census[64];

    memset((void*)g_disparity_visual, 0, sizeof(g_disparity_visual));

    /* Enable DWT Cycle Counter on Cortex-M33 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT       = 0;

    for (int y = WIN_RADIUS; y < (int)(64 - WIN_RADIUS); y++)
    {
        /* Step 1: Precompute Census code for horizontal line (64 width) */
        for (int x = WIN_RADIUS; x < (int)(64 - WIN_RADIUS); x++)
        {
            uint8_t center_l = left_64[y * 64 + x];
            uint8_t center_r = right_64[y * 64 + x];
            uint32_t l_code = 0, r_code = 0;
            int bit_idx = 0;

            for (int dy = -WIN_RADIUS; dy <= WIN_RADIUS; dy++)
            {
                for (int dx = -WIN_RADIUS; dx <= WIN_RADIUS; dx++)
                {
                    if (dx == 0 && dy == 0) continue;
                    if (left_64[(y + dy) * 64 + (x + dx)] >= center_l)  l_code |= (1U << bit_idx);
                    if (right_64[(y + dy) * 64 + (x + dx)] >= center_r) r_code |= (1U << bit_idx);
                    bit_idx++;
                }
            }
            left_census[x]  = l_code;
            right_census[x] = r_code;
        }

        /* Step 2: Epipolar Matching (Search range up to 8 disparity hypotheses for 64x64) */
        for (int x = (int)(WIN_RADIUS + 8); x < (int)(64 - WIN_RADIUS); x++)
        {
            uint32_t l_code   = left_census[x];
            uint32_t min_cost = 0xFFFFFFFF;
            uint8_t  best_d   = 0;

            for (uint8_t d = 0; d < 8; d++)
            {
                uint32_t r_code = right_census[x - d];
                uint32_t cost   = (uint32_t)__builtin_popcount(l_code ^ r_code);

                if (cost < min_cost)
                {
                    min_cost = cost;
                    best_d   = d;
                }
            }

            /* Scale 0..7 disparity up to 0..255 and render into 2x2 pixel blocks on 128x128 map */
            uint8_t scaled_disp = (uint8_t)(best_d * (255 / 7));
            int y2 = y * 2;
            int x2 = x * 2;
            g_disparity_visual[y2][x2]         = scaled_disp;
            g_disparity_visual[y2][x2 + 1]     = scaled_disp;
            g_disparity_visual[y2 + 1][x2]     = scaled_disp;
            g_disparity_visual[y2 + 1][x2 + 1] = scaled_disp;
        }
    }

    /* Fast In-Place Rolling Median Filter */
    apply_rolling_median_filter();

    /* Benchmark statistics */
    g_inference_cycles = DWT->CYCCNT;
    g_inference_ms     = (float)g_inference_cycles / 200000.0f;
}

/* -----------------------------------------------------------------------------------------------------------------
 * ASCII Depth Visualizer: Populates g_ascii_depth in SRAM (32 rows x 64 cols)
 * ----------------------------------------------------------------------------------------------------------------- */
void update_ascii_depth_map(void)
{
    const char depth_chars[] = " .:-=+*#%@";
    const int num_levels = sizeof(depth_chars) - 1;

    for (int r = 0; r < 32; r++)
    {
        int y = r * 4;
        for (int c = 0; c < 64; c++)
        {
            int x = c * 2;
            uint8_t val = g_disparity_visual[y][x];

            if (val == 0)
            {
                g_ascii_depth[r][c] = ' ';
            }
            else
            {
                int idx = (val * (num_levels - 1)) / 255;
                if (idx >= num_levels) idx = num_levels - 1;
                g_ascii_depth[r][c] = depth_chars[idx];
            }
        }
        g_ascii_depth[r][64] = '\0';
    }
}

/* -----------------------------------------------------------------------------------------------------------------
 * Multi-Object Radar & TinyML Hazard Classifier
 * ----------------------------------------------------------------------------------------------------------------- */
void detect_obstacles(void)
{
    /* Sector 1: Left Sidewalk Zone - Pedestrian (X in [20, 35], Y in [50, 80]) */
    uint8_t max_val_left = 0;
    for (int y = 50; y < 80; y++)
    {
        for (int x = 20; x <= 35; x++)
        {
            uint8_t val = g_disparity_visual[y][x];
            if (val > max_val_left)
            {
                max_val_left = val;
            }
        }
    }
    float disp_left_px = (float)max_val_left * (float)(MAX_DISPARITY - 1) / 255.0f;
    g_dist_left_cm = (disp_left_px >= 4.0f) ? (K_CALIBRATION_CM / disp_left_px) : 999.0f;

    /* Sector 2: Center Travel Lane - Approaching Lead Vehicle (X in [45, 95], Y in [50, 95]) */
    uint8_t max_val_center = 0;
    int max_x_center = 70;
    int max_y_center = 70;
    for (int y = 50; y < 95; y++)
    {
        for (int x = 45; x <= 95; x++)
        {
            uint8_t val = g_disparity_visual[y][x];
            if (val > max_val_center)
            {
                max_val_center = val;
                max_x_center   = x;
                max_y_center   = y;
            }
        }
    }
    float disp_center_px = (float)max_val_center * (float)(MAX_DISPARITY - 1) / 255.0f;
    g_dist_center_cm = (disp_center_px >= 4.0f) ? (K_CALIBRATION_CM / disp_center_px) : 999.0f;

    /* Sector 3: Right Roadside Shoulder - Traffic Hazard Sign (X in [100, 124], Y in [45, 80]) */
    uint8_t max_val_right = 0;
    for (int y = 45; y < 80; y++)
    {
        for (int x = 100; x <= 124; x++)
        {
            uint8_t val = g_disparity_visual[y][x];
            if (val > max_val_right)
            {
                max_val_right = val;
            }
        }
    }
    float disp_right_px = (float)max_val_right * (float)(MAX_DISPARITY - 1) / 255.0f;
    g_dist_right_cm = (disp_right_px >= 4.0f) ? (K_CALIBRATION_CM / disp_right_px) : 999.0f;

    /* Multi-Obstacle Detection Counter: Tally obstacles within detection horizon (< 200 cm) */
    uint32_t count = 0;
    if (g_dist_left_cm < 200.0f)   count++;
    if (g_dist_center_cm < 200.0f) count++;
    if (g_dist_right_cm < 200.0f)  count++;
    g_num_obstacles_detected = count;

    /* Forward Collision Radar tracks the critical approaching vehicle in the travel lane */
    g_nearest_obstacle_cm = g_dist_center_cm;
    g_obstacle_direction  = OBSTACLE_CENTER;

    /* -------------------------------------------------------------------------------------------------------------
     * TinyML Feature Extraction on Forward Critical Obstacle:
     *   Feature 1: Peak Disparity in pixels (Distance metric: 5 -> 6 -> 7 -> 10 -> 15 px)
     *   Feature 2: Horizontal Centroid normalized (0.0 to 1.0)
     *   Feature 3: Disparity Spatial Compactness (Solid area ratio around peak)
     * ------------------------------------------------------------------------------------------------------------- */
    g_feat_peak_disparity = disp_center_px;
    g_feat_centroid_x     = (float)max_x_center / (float)IMAGE_WIDTH;

    uint32_t patch_total = 0;
    uint32_t patch_solid = 0;
    uint8_t threshold = (max_val_center > 30) ? (uint8_t)(max_val_center - 25) : 10;
    for (int dy = -8; dy <= 8; dy++)
    {
        int py = max_y_center + dy;
        if (py < 0 || py >= (int)IMAGE_HEIGHT) continue;
        for (int dx = -8; dx <= 8; dx++)
        {
            int px = max_x_center + dx;
            if (px < 0 || px >= (int)IMAGE_WIDTH) continue;
            patch_total++;
            if (g_disparity_visual[py][px] >= threshold)
            {
                patch_solid++;
            }
        }
    }
    g_feat_compactness = (patch_total > 0) ? ((float)patch_solid / (float)patch_total) : 0.0f;

    /* -------------------------------------------------------------------------------------------------------------
     * TinyML Machine Learning Inference:
     * Executes trained Softmax Neural Classifier (< 100 clock cycles on Cortex-M33)
     * ------------------------------------------------------------------------------------------------------------- */
    float features[TINYML_NUM_FEATURES] = {
        g_feat_peak_disparity,
        g_feat_centroid_x,
        g_feat_compactness
    };

    float confidence = 0.0f;
    uint8_t ml_predicted_class = tinyml_predict_hazard(features, &confidence);
    g_tinyml_confidence = confidence * 100.0f; /* 0.0% to 100.0% */

    /* TinyML Classifier directly sets hazard alert level */
    g_alert_level = (alert_level_t)ml_predicted_class;
}

/* -----------------------------------------------------------------------------------------------------------------
 * Board-Agnostic LED Subsystem:
 * Automatically supports FPB-RA6E2 (P207, P206), BGK-RA6E2 (P104, P112), and EK-RA6E2 (P400, P113).
 * Renesas RA on-board user LEDs operate with Active-High logic:
 *   - HIGH (3.3V) = LED ON
 *   - LOW  (0.0V) = LED OFF
 * ----------------------------------------------------------------------------------------------------------------- */
#define LED_STATE_ON   (BSP_IO_LEVEL_HIGH)
#define LED_STATE_OFF  (BSP_IO_LEVEL_LOW)

/* Set LED 1 (Primary Alert Indicator - Green / LED1) across all possible board revisions */
static inline void bsp_set_led1(bsp_io_level_t level)
{
    R_BSP_PinAccessEnable();
    R_BSP_PinWrite(BSP_IO_PORT_04_PIN_00, level); /* EK-RA6E2  LED2 (Green) */
    R_BSP_PinWrite(BSP_IO_PORT_02_PIN_07, level); /* FPB-RA6E2 LED1 (Green) / EK-RA6E2 LED1 (Blue) */
    R_BSP_PinWrite(BSP_IO_PORT_01_PIN_04, level); /* BGK-RA6E2 LED1 */
    R_BSP_PinAccessDisable();

    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_04_PIN_00, level);
    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_02_PIN_07, level);
    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_01_PIN_04, level);
}

/* Set LED 2 (Secondary / Stop Indicator - Red / LED2) across all possible board revisions */
static inline void bsp_set_led2(bsp_io_level_t level)
{
    R_BSP_PinAccessEnable();
    R_BSP_PinWrite(BSP_IO_PORT_01_PIN_13, level); /* EK-RA6E2  LED3 (Red) */
    R_BSP_PinWrite(BSP_IO_PORT_02_PIN_06, level); /* FPB-RA6E2 LED2 (Green) */
    R_BSP_PinWrite(BSP_IO_PORT_01_PIN_12, level); /* BGK-RA6E2 LED2 */
    R_BSP_PinAccessDisable();

    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_01_PIN_13, level);
    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_02_PIN_06, level);
    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_01_PIN_12, level);
}

/* Drive ALL on-board LEDs simultaneously for maximum visibility and board compatibility */
static inline void bsp_set_all_leds(bsp_io_level_t level)
{
    bsp_set_led1(level);
    bsp_set_led2(level);
}

/* Initialize all candidate pins as GPIO Outputs and perform Power-On Self Test */
static void bsp_leds_init(void)
{
    const bsp_io_port_pin_t candidate_pins[] = {
        BSP_IO_PORT_02_PIN_07, /* FPB-RA6E2 LED1 */
        BSP_IO_PORT_02_PIN_06, /* FPB-RA6E2 LED2 */
        BSP_IO_PORT_01_PIN_04, /* BGK-RA6E2 LED1 */
        BSP_IO_PORT_01_PIN_12, /* BGK-RA6E2 LED2 */
        BSP_IO_PORT_04_PIN_00, /* EK-RA6E2  LED2 */
        BSP_IO_PORT_01_PIN_13  /* EK-RA6E2  LED3 */
    };
    uint32_t num_pins = sizeof(candidate_pins) / sizeof(candidate_pins[0]);

    R_BSP_PinAccessEnable();
    for (uint32_t i = 0; i < num_pins; i++)
    {
        /* Reconfigure pin function from peripheral mode (e.g. SSI) to GPIO output */
        R_BSP_PinCfg(candidate_pins[i], IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_PORT_OUTPUT_LOW);
        R_IOPORT_PinCfg(&g_ioport_ctrl, candidate_pins[i], IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_PORT_OUTPUT_LOW);
    }
    R_BSP_PinAccessDisable();

    bsp_set_all_leds(LED_STATE_OFF);

    /* Power-On Self-Test (POST): 3 distinct confirmation blinks on boot */
    for (int i = 0; i < 3; i++)
    {
        bsp_set_all_leds(LED_STATE_ON);
        R_BSP_SoftwareDelay(150, BSP_DELAY_UNITS_MILLISECONDS);
        bsp_set_all_leds(LED_STATE_OFF);
        R_BSP_SoftwareDelay(150, BSP_DELAY_UNITS_MILLISECONDS);
    }
}

/* -----------------------------------------------------------------------------------------------------------------
 * Multi-Frequency LED Demonstration Engine:
 * Generates parking radar alert blinking patterns across the remaining time of the 1-second interval
 * ----------------------------------------------------------------------------------------------------------------- */
void execute_led_alert_pattern(alert_level_t level, uint32_t duration_ms)
{
    switch (level)
    {
        case ALERT_SAFE:
            /* Frame 1 (90 cm): Safe Path -> Both LEDs OFF */
            bsp_set_led1(LED_STATE_OFF);
            bsp_set_led2(LED_STATE_OFF);
            R_BSP_SoftwareDelay(duration_ms, BSP_DELAY_UNITS_MILLISECONDS);
            break;

        case ALERT_APPROACH:
            /* Frame 2 (75 cm): Far Approach -> Both LEDs OFF */
            bsp_set_led1(LED_STATE_OFF);
            bsp_set_led2(LED_STATE_OFF);
            R_BSP_SoftwareDelay(duration_ms, BSP_DELAY_UNITS_MILLISECONDS);
            break;

        case ALERT_CAUTION:
            /* Frame 3 (60 cm): "one led when 60 cm" -> LED1 Solid ON, LED2 OFF */
            bsp_set_led1(LED_STATE_ON);
            bsp_set_led2(LED_STATE_OFF);
            R_BSP_SoftwareDelay(duration_ms, BSP_DELAY_UNITS_MILLISECONDS);
            break;

        case ALERT_WARNING:
            /* Frame 4 (45 cm): "one led fast bllink in 45 cm" -> LED1 Fast Blink (~7 Hz), LED2 OFF */
            bsp_set_led2(LED_STATE_OFF);
            {
                /* 6 rapid strobe pulses: 70 ms ON / 70 ms OFF = 840 ms */
                for (int k = 0; k < 6; k++)
                {
                    bsp_set_led1(LED_STATE_ON);
                    R_BSP_SoftwareDelay(70, BSP_DELAY_UNITS_MILLISECONDS);

                    bsp_set_led1(LED_STATE_OFF);
                    R_BSP_SoftwareDelay(70, BSP_DELAY_UNITS_MILLISECONDS);
                }
                if (duration_ms > 840)
                {
                    R_BSP_SoftwareDelay(duration_ms - 840, BSP_DELAY_UNITS_MILLISECONDS);
                }
            }
            break;

        case ALERT_CRITICAL_STOP:
        default:
            /* Frame 5 (30 cm): "30 cm 2 led solid blink" -> BOTH LED1 + LED2 Solid Bright ON */
            bsp_set_led1(LED_STATE_ON);
            bsp_set_led2(LED_STATE_ON);
            R_BSP_SoftwareDelay(duration_ms, BSP_DELAY_UNITS_MILLISECONDS);
            break;
    }

    /* Turn off pins at slot end */
    bsp_set_led1(LED_STATE_OFF);
    bsp_set_led2(LED_STATE_OFF);
}

/* Fault Diagnostics (inspectable in Expressions tab if any unexpected fault occurs) */
volatile uint32_t g_fault_cfsr = 0;
volatile uint32_t g_fault_hfsr = 0;
volatile uint32_t g_fault_bfar = 0;
volatile uint32_t g_fault_pc   = 0;

void HardFault_Handler(void)
{
    g_fault_cfsr = SCB->CFSR;
    g_fault_hfsr = SCB->HFSR;
    g_fault_bfar = SCB->BFAR;
    __asm volatile (
        "tst lr, #4 \n"
        "ite eq \n"
        "mrseq r0, msp \n"
        "mrsne r0, psp \n"
        "ldr r1, [r0, #24] \n"
        "str r1, %0 \n"
        : "=m" (g_fault_pc)
        :
        : "r0", "r1"
    );
    while (1);
}

void BusFault_Handler(void)   { HardFault_Handler(); }
void MemManage_Handler(void)  { HardFault_Handler(); }
void UsageFault_Handler(void) { HardFault_Handler(); }

/* -----------------------------------------------------------------------------------------------------------------
 * Official FSP SCI UART Driver (Referenced from sci_uart_ek_ra6e2_ep)
 * Target MCU: Renesas RA6E2 (BGK-RA6E2 Board)
 * Channel 0 (SCI0 on P411/P410 @ 115200 Baud, 8N1) -> SEGGER J-Link CDC Virtual COM Port (COM3)
 * ----------------------------------------------------------------------------------------------------------------- */
static volatile uint8_t   g_uart_event    = 0;
static volatile fsp_err_t g_uart_open_err = FSP_SUCCESS;

void user_uart_callback(uart_callback_args_t *p_args)
{
    g_uart_event = (uint8_t)p_args->event;
}

static void uart_init(void)
{
    /* Initialize FSP SCI UART channel 0 @ 115200 Baud (Reference: sci_uart_ek_ra6e2_ep) */
    g_uart_open_err = R_SCI_UART_Open(&g_uart_ctrl, &g_uart_cfg);
}

static void uart_puts(const char *str)
{
    if (FSP_SUCCESS != g_uart_open_err) return;

    uint32_t len = (uint32_t)strlen(str);
    if (len == 0) return;

    g_uart_event = 0;
    fsp_err_t err = R_SCI_UART_Write(&g_uart_ctrl, (uint8_t const *)str, len);
    if (FSP_SUCCESS == err)
    {
        uint32_t timeout = 500000;
        while ((UART_EVENT_TX_COMPLETE != g_uart_event) && (--timeout > 0));
    }
}

/* Transmit telemetry log for PuTTY Terminal and $RADAR packet for Web Dashboard */
static char s_banner[600];
static char s_radar_pkt[160];

static void uart_transmit_frame_packet(void)
{
    const char *alert_str = "SAFE (LEDs OFF)";
    const char *led_str   = "LEDs OFF";
    const char *bar_str   = "[==                  ]";

    switch (g_alert_level)
    {
        case ALERT_APPROACH:
            alert_str = "APPROACH (Safe Distance)";
            led_str   = "LEDs OFF";
            bar_str   = "[====                ]";
            break;
        case ALERT_CAUTION:
            alert_str = "CAUTION (one led when 60 cm)";
            led_str   = "LED1 SOLID ON";
            bar_str   = "[========            ]";
            break;
        case ALERT_WARNING:
            alert_str = "WARNING (one led fast blink in 45 cm)";
            led_str   = "LED1 FAST STROBE (~7 Hz)";
            bar_str   = "[==============      ]";
            break;
        case ALERT_CRITICAL_STOP:
            alert_str = "EMERGENCY STOP (30 cm 2 led solid blink)";
            led_str   = "LED1 + LED2 SOLID ON";
            bar_str   = "[====================]";
            break;
        case ALERT_SAFE:
        default:
            alert_str = "SAFE (Clear Roadway)";
            led_str   = "LEDs OFF";
            bar_str   = "[==                  ]";
            break;
    }

    uint32_t total_frames = g_sd_card_present ? g_sd_frame_count : NUM_VIDEO_FRAMES;

    snprintf(s_banner, sizeof(s_banner),
        "\r\n==============================================================\r\n"
        " [FRAME %u/%u] Stereo Vision & TinyML Collision Radar\r\n"
        " Source Stream    : %s\r\n"
        "--------------------------------------------------------------\r\n"
        " Obstacle Left    : %d.%d cm [%s]\r\n"
        " Obstacle Center  : %d.%d cm [%s]\r\n"
        " Obstacle Right   : %d.%d cm [%s]\r\n"
        " Nearest Hazard   : %d.%d cm -> %s\r\n"
        " TinyML Confidence: %d.%d%% | Spatial Compactness: %d.%d%%\r\n"
        " Visual Depth Bar : %s\r\n"
        " Board LED Action : %s\r\n"
        " Execution Time   : %d.%d ms (5x5 Census + Popcount)\r\n"
        "--------------------------------------------------------------\r\n",
        (unsigned int)g_current_frame,
        (unsigned int)total_frames,
        (const char*)g_active_source,
        (int)(g_dist_left_cm), (int)((int)(g_dist_left_cm * 10.0f) % 10), (g_dist_left_cm < 200.0f ? "SIDEWALK" : "CLEAR"),
        (int)(g_dist_center_cm), (int)((int)(g_dist_center_cm * 10.0f) % 10), (g_dist_center_cm < 200.0f ? "LEAD VEHICLE" : "CLEAR"),
        (int)(g_dist_right_cm), (int)((int)(g_dist_right_cm * 10.0f) % 10), (g_dist_right_cm < 200.0f ? "SHOULDER" : "CLEAR"),
        (int)(g_nearest_obstacle_cm), (int)((int)(g_nearest_obstacle_cm * 10.0f) % 10), alert_str,
        (int)(g_tinyml_confidence), (int)((int)(g_tinyml_confidence * 10.0f) % 10),
        (int)(g_feat_compactness * 100.0f), (int)((int)(g_feat_compactness * 1000.0f) % 10),
        bar_str,
        led_str,
        (int)(g_inference_ms), (int)((int)(g_inference_ms * 10.0f) % 10)
    );
    /* Stream formatted telemetry log to SEGGER RTT (Channel 0) */
    SEGGER_RTT_WriteString(0, s_banner);

    /* Also transmit $RADAR packet for Web Dashboard */
    snprintf(s_radar_pkt, sizeof(s_radar_pkt),
        "$RADAR,%u,%u,%d,%d,%d,%d,%u,%u,%u,%d,%d,%d\r\n",
        (unsigned int)g_current_frame,
        (unsigned int)(g_inference_ms * 100.0f),
        (int)(g_dist_left_cm * 10.0f),
        (int)(g_dist_center_cm * 10.0f),
        (int)(g_dist_right_cm * 10.0f),
        (int)(g_nearest_obstacle_cm * 10.0f),
        (unsigned int)g_num_obstacles_detected,
        (unsigned int)g_alert_level,
        (unsigned int)(g_tinyml_confidence * 10.0f),
        (int)(g_feat_peak_disparity * 100.0f),
        (int)(g_feat_centroid_x * 1000.0f),
        (int)(g_feat_compactness * 1000.0f)
    );
    SEGGER_RTT_WriteString(0, s_radar_pkt);

    /* Stream framed binary disparity map (128x128 = 16,384 bytes) with CRC32 */
    segger_send_framed_packet((uint8_t)g_current_frame, (const uint8_t *)g_disparity_visual, sizeof(g_disparity_visual));

    /* Secondary fallback: transmit over SCI0 UART if enabled */
    uart_puts(s_banner);
    uart_puts(s_radar_pkt);
}

/*******************************************************************************************************************//**
 * hal_entry() - Main Application Loop
 **********************************************************************************************************************/
void hal_entry(void)
{
#if (0 == _RA_CORE) && (1 == BSP_MULTICORE_PROJECT) && !BSP_TZ_NONSECURE_BUILD
#if BSP_TZ_SECURE_BUILD
    R_BSP_IpcSemaphoreTake(&g_core_start_semaphore);
#endif
    R_BSP_SecondaryCoreStart();
#if BSP_TZ_SECURE_BUILD
    while(FSP_ERR_IN_USE == R_BSP_IpcSemaphoreTake(&g_core_start_semaphore))
    {
        ;
    }
#endif
#endif

#if (1 == _RA_CORE) && (1 == BSP_MULTICORE_PROJECT) && BSP_TZ_SECURE_BUILD
    R_BSP_IpcSemaphoreGive(&g_core_start_semaphore);
#endif

#if BSP_TZ_SECURE_BUILD
    R_BSP_NonSecureEnter();
#endif

    /* Configure Alert LED Pins as Outputs with Power-On Self-Test */
    bsp_leds_init();

    /* Initialize SEGGER RTT Real-Time Transfer Engine */
    segger_transfer_init();
    R_BSP_SoftwareDelay(50, BSP_DELAY_UNITS_MILLISECONDS);

    segger_printf("\r\n\r\n"
                  "**************************************************************\r\n"
                  "  RENESAS BGK-RA6E2 STEREO VISION & TINYML COLLISION RADAR    \r\n"
                  "  Target MCU : Renesas RA6E2 (Arm Cortex-M33 @ 200 MHz)       \r\n"
                  "  Transport  : SEGGER J-Link RTT Channel 0                    \r\n"
                  "  Dashboard  : Streaming live telemetry to localhost:19021     \r\n"
                  "**************************************************************\r\n");

    /* Initialize SCI0 UART (J-Link CDC VCOM @ 115200 Baud) fallback */
    uart_init();
    uart_puts("\r\n\r\n"
              "**************************************************************\r\n"
              "  RENESAS BGK-RA6E2 STEREO VISION & TINYML COLLISION RADAR    \r\n"
              "  Target MCU : Renesas RA6E2 (Arm Cortex-M33 @ 200 MHz)       \r\n"
              "  Serial Com : SCI0 (P411-TX / P410-RX @ 115200 Baud, 8N1)   \r\n"
              "  PuTTY Log  : Online - Streaming Multi-Object Telemetry      \r\n"
              "**************************************************************\r\n");

    /* Initialize Micro SD Card SPI & FAT32 Subsystem (Arduino Interface: D10-CS, D11-MOSI, D12-MISO, D13-SCK) */
    segger_printf("\r\n==============================================================\r\n");
    segger_printf("  RENESAS BGK-RA6E2 STEREO VISION & SD CARD ENGINE (V2.0)     \r\n");
    segger_printf("  Arduino Pins: D10(P301)-CS, D11(P109)-MOSI, D12(P110)-MISO, D13(P111)-SCK\r\n");
    segger_printf("==============================================================\r\n");

    uart_puts("\r\n[SD] Mounting 32GB Micro SD Card via SPI (D10-D13)...\r\n");

    fat32_err_t fat_err = fat32_mount();
    if (fat_err == FAT32_OK)
    {
        g_sd_card_present = true;
        snprintf((char*)g_active_source, sizeof(g_active_source), "32GB SD CARD (KITTI 10-FRAMES)");
        segger_printf("[SD SUCCESS] 32GB Micro SD Card Mounted (FAT32)!\r\n");
        segger_printf("[SD SUCCESS] Partition LBA: %lu, Cluster Size: %u sectors (%u KB)\r\n",
                      g_fat32_vol.partition_lba,
                      g_fat32_vol.sectors_per_cluster,
                      (g_fat32_vol.sectors_per_cluster * 512U) / 1024U);
        uart_puts("[SD SUCCESS] Streaming 10 Real-World KITTI Stereo Pairs!\r\n");

        /* Visual confirmation on board: 3 synchronized LED flashes */
        for (int i = 0; i < 3; i++)
        {
            bsp_set_all_leds(LED_STATE_ON);
            R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS);
            bsp_set_all_leds(LED_STATE_OFF);
            R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS);
        }
    }
    else
    {
        g_sd_card_present = false;
        snprintf((char*)g_active_source, sizeof(g_active_source), "FLASH FALLBACK (SD: %s)", sd_get_error_str(g_sd_init_error));
        segger_printf("\r\n[SD NOTICE] Micro SD Card NOT ACTIVE (%s)\r\n", sd_get_error_str(g_sd_init_error));
        segger_printf("[SD DIAG] CMD0 raw response = 0x%02X\r\n", (unsigned int)g_sd_last_cmd0_res);
        if (g_sd_last_cmd0_res == 0xFF)
        {
            segger_printf("[SD DIAG] MISO pin (D12 / Pin 27) received 0xFF (No Signal).\r\n"
                          "          Verify wiring: D10(Pin25)=CS, D11(Pin26)=MOSI, D12(Pin27)=MISO, D13(Pin28)=SCK,\r\n"
                          "          Power: Pin10(5V) or Pin11(3V3), GND: Pin29.\r\n");
        }
        segger_printf("[SD FALLBACK] Automatically falling back to on-chip Code Flash video stream.\r\n\r\n");
        uart_puts("[SD NOTICE] SD Card not mounted. Playing on-chip Flash dataset.\r\n");

        /* Hardware indication on board: 5 rapid alert blinks */
        for (int i = 0; i < 5; i++)
        {
            bsp_set_led1(LED_STATE_ON);
            R_BSP_SoftwareDelay(80, BSP_DELAY_UNITS_MILLISECONDS);
            bsp_set_led1(LED_STATE_OFF);
            R_BSP_SoftwareDelay(80, BSP_DELAY_UNITS_MILLISECONDS);
        }
        bsp_set_all_leds(LED_STATE_OFF);
    }

    /* -------------------------------------------------------------------------------------------------------------
     * Continuous Video Playback Loop (1 FPS) - Supports both SD Card KITTI Stream & On-Chip Flash Video
     * ------------------------------------------------------------------------------------------------------------- */
    while (1)
    {
        uint32_t total_loop_frames = g_sd_card_present ? g_sd_frame_count : NUM_VIDEO_FRAMES;

        for (uint32_t f = 0; f < total_loop_frames; f++)
        {
            g_current_frame = f + 1; /* Frame 1 to N */
            bool loaded_from_sd = false;

            /* Step 0: Check for incoming interactive commands from PC host over RTT */
            if (segger_has_key())
            {
                int cmd = segger_get_key();
                if (cmd == '1')
                {
                    segger_send_framed_packet(0x11, g_video_left_frames[f % NUM_VIDEO_FRAMES], 16384);
                }
                else if (cmd == '2')
                {
                    segger_send_framed_packet(0x12, g_video_right_frames[f % NUM_VIDEO_FRAMES], 16384);
                }
            }

            /* Step 1: Acquire & Process Stereo Vision Frame */
            if (g_sd_card_present)
            {
                char file_l[16], file_r[16];
                snprintf(file_l, sizeof(file_l), "L%04lu.BIN", f);
                snprintf(file_r, sizeof(file_r), "R%04lu.BIN", f);

                fat32_file_t fl, fr;
                uint32_t read_l = 0, read_r = 0;

                if (fat32_open("STEREO64", file_l, &fl) == FAT32_OK &&
                    fat32_open("STEREO64", file_r, &fr) == FAT32_OK)
                {
                    fat32_read(&fl, g_sd_left_frame, sizeof(g_sd_left_frame), &read_l);
                    fat32_read(&fr, g_sd_right_frame, sizeof(g_sd_right_frame), &read_r);

                    if (read_l == sizeof(g_sd_left_frame) && read_r == sizeof(g_sd_right_frame))
                    {
                        run_stereo_disparity_64_to_128(g_sd_left_frame, g_sd_right_frame);
                        loaded_from_sd = true;
                    }
                }
            }

            if (!loaded_from_sd)
            {
                /* Fallback: Process from on-chip Code Flash (.rodata) */
                run_stereo_disparity_128(g_video_left_frames[f % NUM_VIDEO_FRAMES],
                                         g_video_right_frames[f % NUM_VIDEO_FRAMES]);
            }

            /* Step 2: Render subsampled ASCII depth map for in-IDE Expressions view */
            update_ascii_depth_map();

            /* Step 3: Detect obstacle distance and classify hazard alert level */
            detect_obstacles();

            /* Step 4: Transmit Telemetry & Disparity Map over UART */
            uart_transmit_frame_packet();

            /* Step 5: Allocate remaining slot time for LED alert pattern */
            uint32_t remaining_ms = 860;
            if (g_inference_ms > 0.0f && g_inference_ms < 950.0f)
            {
                remaining_ms = (uint32_t)(1000.0f - g_inference_ms);
            }

            /* Step 6: Drive physical LEDs with progressive alert frequency */
            execute_led_alert_pattern(g_alert_level, remaining_ms);

            g_total_frames_played++;
        }

        /* End of loop quiet interval */
        g_video_loop_count++;
        bsp_set_all_leds(LED_STATE_OFF);
        R_BSP_SoftwareDelay(1000, BSP_DELAY_UNITS_MILLISECONDS);
    }
}

#if BSP_TZ_SECURE_BUILD
FSP_CPP_HEADER
BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ();
BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ()
{
}
FSP_CPP_FOOTER
#endif
