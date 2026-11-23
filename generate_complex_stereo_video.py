import os
import numpy as np
from PIL import Image, ImageDraw, ImageFont

H, W = 128, 128
MAX_DISPARITY = 16
WIN_RADIUS = 2

# Fixed seed for deterministic, crisp textures
np.random.seed(42)
car_tex_master = np.random.randint(50, 220, size=(128, 128), dtype=np.uint8)
ped_tex_master = np.random.randint(40, 200, size=(128, 128), dtype=np.uint8)
sign_tex_master = np.random.randint(60, 240, size=(128, 128), dtype=np.uint8)

def render_car(w, h):
    mask = Image.new('L', (w, h), 0)
    dm = ImageDraw.Draw(mask)
    cw = int(w * 0.7)
    cx0 = (w - cw) // 2
    ch = int(h * 0.45)
    dm.rectangle([cx0, 0, cx0 + cw, ch], fill=255)
    dm.rectangle([0, int(ch * 0.7), w, h], fill=255)
    ww = max(2, int(w * 0.18))
    dm.rectangle([1, h - 3, 1 + ww, h], fill=0)
    dm.rectangle([w - 1 - ww, h - 3, w - 1, h], fill=0)
    tex = Image.fromarray(car_tex_master[:h, :w])
    res = Image.composite(tex, Image.new('L', (w, h), 0), mask)
    return np.array(res), np.array(mask)

def render_pedestrian(h):
    w = max(6, int(h * 0.4))
    mask = Image.new('L', (w, h), 0)
    dm = ImageDraw.Draw(mask)
    hr = max(2, int(h * 0.12))
    dm.ellipse([w//2 - hr, 0, w//2 + hr, 2*hr], fill=255)
    dm.rectangle([w//2 - int(w*0.35), 2*hr, w//2 + int(w*0.35), int(h*0.65)], fill=255)
    dm.rectangle([w//2 - int(w*0.35), int(h*0.65), w//2 - 1, h], fill=255)
    dm.rectangle([w//2 + 1, int(h*0.65), w//2 + int(w*0.35), h], fill=255)
    tex = Image.fromarray(ped_tex_master[:h, :w])
    res = Image.composite(tex, Image.new('L', (w, h), 0), mask)
    return np.array(res), np.array(mask)

def render_sign(size):
    mask = Image.new('L', (size, size + 16), 0)
    dm = ImageDraw.Draw(mask)
    half = size // 2
    pts = [(half, 0), (size, half), (half, size), (0, half)]
    dm.polygon(pts, fill=255)
    dm.rectangle([half - 1, size, half + 1, size + 16], fill=255)
    tex = Image.fromarray(sign_tex_master[:size+16, :size])
    res = Image.composite(tex, Image.new('L', (size, size + 16), 0), mask)
    return np.array(res), np.array(mask)

def compute_census_image(img):
    census = np.zeros((H, W), dtype=np.uint32)
    for y in range(WIN_RADIUS, H - WIN_RADIUS):
        for x in range(WIN_RADIUS, W - WIN_RADIUS):
            center = int(img[y, x])
            code = 0
            bit = 0
            for dy in range(-WIN_RADIUS, WIN_RADIUS + 1):
                for dx in range(-WIN_RADIUS, WIN_RADIUS + 1):
                    if dx == 0 and dy == 0:
                        continue
                    if int(img[y + dy, x + dx]) >= center:
                        code |= (1 << bit)
                    bit += 1
            census[y, x] = code
    return census

def popcount(x):
    return bin(x).count('1')

def run_stereo(left, right):
    lc = compute_census_image(left)
    rc = compute_census_image(right)
    disp = np.zeros((H, W), dtype=np.uint8)
    for y in range(WIN_RADIUS, H - WIN_RADIUS):
        for x in range(WIN_RADIUS + MAX_DISPARITY, W - WIN_RADIUS):
            l_val = lc[y, x]
            min_cost = 999
            best_d = 0
            for d in range(MAX_DISPARITY):
                r_val = rc[y, x - d]
                cost = popcount(l_val ^ r_val)
                if cost < min_cost:
                    min_cost = cost
                    best_d = d
            disp[y, x] = int(best_d * (255 // (MAX_DISPARITY - 1)))
    # 3x3 rolling median filter
    disp_filt = np.copy(disp)
    for y in range(1, H - 1):
        for x in range(WIN_RADIUS + MAX_DISPARITY + 1, W - 1):
            disp_filt[y, x] = int(np.median(disp[y-1:y+2, x-1:x+2]))
    return disp_filt

# Dynamic multi-object configuration across 5 frames:
# Frame 1: 2 obstacles (Car @ 90cm + Sign @ 90cm, Pedestrian not yet in view)
# Frame 2: 3 obstacles (Pedestrian @ 90cm + Car @ 75cm + Sign @ 75cm)
# Frame 3: 3 obstacles (Pedestrian @ 75cm + Car @ 64.3cm + Sign alongside @ 50cm)
# Frame 4: 2 obstacles (Pedestrian @ 56.2cm + Car @ 45cm, Sign passed out of view)
# Frame 5: 1 obstacle  (Car @ 30cm, Pedestrian & Sign both passed behind)

frame_configs = [
    # (frame_idx, car_disp, car_w, car_h, car_y, target_cm, ped_cfg, sign_cfg)
    (0,  5, 18, 15, 60, 90.0, None,                   (5, 12, 106, 54)),
    (1,  6, 22, 18, 63, 75.0, (5, 22, 26, 55),        (6, 14, 106, 52)),
    (2,  7, 26, 22, 67, 64.3, (6, 25, 26, 54),        (9, 18, 108, 48)),
    (3, 10, 32, 26, 71, 45.0, (8, 28, 25, 53),        None),
    (4, 15, 40, 32, 77, 30.0, None,                   None),
]

left_frames = []
right_frames = []
disp_frames = []

for f_id, car_disp, car_w, car_h, car_y, target_cm, ped_cfg, sign_cfg in frame_configs:
    l_im = np.full((H, W), 45, dtype=np.uint8)
    r_im = np.full((H, W), 45, dtype=np.uint8)
    
    # Sky and distant scenery
    l_im[:42, :] = 175
    r_im[:42, :] = 175
    for x in range(0, W, 8):
        tree_h = 10 + (x * 7) % 15
        l_im[42 - tree_h:42, x:x+6] = 70 + (x * 13) % 40
        r_im[42 - tree_h:42, x:x+6] = 70 + (x * 13) % 40
        
    # Road gradient
    for y in range(42, H):
        l_im[y, :] = 50 + int((y - 42) * 0.45)
        r_im[y, :] = 50 + int((y - 42) * 0.45)
        # Road lane dashes (moving forward)
        if (y + f_id * 6) % 18 < 9:
            l_im[y, 63:65] = 220
            r_im[y, 63:65] = 220
            
    # Object 1 (Left Zone): Pedestrian on sidewalk
    if ped_cfg is not None:
        p_disp, p_h, px_l, py = ped_cfg
        ped_im, ped_m = render_pedestrian(p_h)
        ph, pw = ped_im.shape
        l_im[py:py+ph, px_l:px_l+pw] = np.where(ped_m > 0, ped_im, l_im[py:py+ph, px_l:px_l+pw])
        px_r = px_l - p_disp
        r_im[py:py+ph, px_r:px_r+pw] = np.where(ped_m > 0, ped_im, r_im[py:py+ph, px_r:px_r+pw])
    
    # Object 3 (Right Zone): Traffic Hazard Sign
    if sign_cfg is not None:
        s_disp, s_size, sx_l, sy = sign_cfg
        sgn_im, sgn_m = render_sign(s_size)
        sh, sw = sgn_im.shape
        l_im[sy:sy+sh, sx_l:sx_l+sw] = np.where(sgn_m > 0, sgn_im, l_im[sy:sy+sh, sx_l:sx_l+sw])
        sx_r = sx_l - s_disp
        r_im[sy:sy+sh, sx_r:sx_r+sw] = np.where(sgn_m > 0, sgn_im, r_im[sy:sy+sh, sx_r:sx_r+sw])
    
    # Object 2 (Center Zone): Approaching Lead Vehicle
    car_im, car_m = render_car(car_w, car_h)
    ch, cw = car_im.shape
    cx_l, cy = 70 - cw//2, car_y - ch//2
    l_im[cy:cy+ch, cx_l:cx_l+cw] = np.where(car_m > 0, car_im, l_im[cy:cy+ch, cx_l:cx_l+cw])
    cx_r = cx_l - car_disp
    r_im[cy:cy+ch, cx_r:cx_r+cw] = np.where(car_m > 0, car_im, r_im[cy:cy+ch, cx_r:cx_r+cw])
    
    left_frames.append(l_im)
    right_frames.append(r_im)
    
    disp = run_stereo(l_im, r_im)
    disp_frames.append(disp)
    
    disp_int = disp.astype(int)
    d_ped = disp_int[50:80, 20:36].max() * 15 // 255
    d_sign = disp_int[45:80, 100:125].max() * 15 // 255
    d_car = disp_int[cy:cy+ch, cx_l:cx_l+cw].max() * 15 // 255
    
    dist_p = f"{450.0/d_ped:.1f}cm" if d_ped >= 1 else "None (999cm)"
    dist_s = f"{450.0/d_sign:.1f}cm" if d_sign >= 1 else "None (999cm)"
    dist_c = f"{450.0/d_car:.1f}cm" if d_car >= 1 else "None (999cm)"
    print(f"Frame {f_id+1}: Target {target_cm}cm -> Car={dist_c} (d={d_car}), Ped={dist_p} (d={d_ped}), Sign={dist_s} (d={d_sign})")

# ----------------------------------------------------------------------------------------------------------------------
# Generate image_data.h
# ----------------------------------------------------------------------------------------------------------------------
header_code = """/*******************************************************************************************************************
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
"""

with open("ra6e2_sv/src/image_data.h", "w") as f:
    f.write(header_code)
print("Updated ra6e2_sv/src/image_data.h")

# ----------------------------------------------------------------------------------------------------------------------
# Generate image_data.c
# ----------------------------------------------------------------------------------------------------------------------
with open("ra6e2_sv/src/image_data.c", "w") as f:
    f.write("/*******************************************************************************************************************\n")
    f.write(" * @file    image_data.c\n")
    f.write(" * @brief   Complex Multi-Object 10-Frame (5 Left + 5 Right) 128x128 Video Sequence.\n")
    f.write(" *          Stored in RA6E2 Internal Code Flash (.rodata): 160 KB total (0 bytes RAM).\n")
    f.write(" *******************************************************************************************************************/\n\n")
    f.write('#include "image_data.h"\n\n')

    dists = [
        "Frame 1: Car @ 90.0 cm, Ped @ None (999 cm), Sign @ 90.0 cm (Count=2)",
        "Frame 2: Car @ 75.0 cm, Ped @ 90.0 cm, Sign @ 75.0 cm (Count=3)",
        "Frame 3: Car @ 64.3 cm, Ped @ 75.0 cm, Sign @ 50.0 cm (Count=3)",
        "Frame 4: Car @ 45.0 cm, Ped @ 56.2 cm, Sign @ None (999 cm) (Count=2)",
        "Frame 5: Car @ 30.0 cm, Ped @ None (999 cm), Sign @ None (999 cm) (Count=1)"
    ]

    f.write("/* 5 Dynamic Left Frames (5 x 16,384 = 81,920 bytes in Flash) */\n")
    f.write('const uint8_t g_video_left[NUM_VIDEO_FRAMES][IMAGE_SIZE_BYTES] __attribute__((used, section(".rodata"), aligned(4))) = {\n')
    for frame_idx, lf in enumerate(left_frames):
        f.write(f"    /* Left {dists[frame_idx]} */\n    {{\n")
        for row in range(128):
            row_bytes = lf[row, :]
            hex_str = ", ".join(f"0x{b:02X}" for b in row_bytes)
            f.write(f"        /* Row {row:03d} */ {hex_str},\n")
        f.write("    },\n")
    f.write("};\n\n")

    f.write("/* 5 Dynamic Right Frames (5 x 16,384 = 81,920 bytes in Flash) */\n")
    f.write('const uint8_t g_video_right[NUM_VIDEO_FRAMES][IMAGE_SIZE_BYTES] __attribute__((used, section(".rodata"), aligned(4))) = {\n')
    for frame_idx, rf in enumerate(right_frames):
        f.write(f"    /* Right {dists[frame_idx]} */\n    {{\n")
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

print("Updated ra6e2_sv/src/image_data.c")

# ----------------------------------------------------------------------------------------------------------------------
# Generate Composite Visual Preview Artifact (5 Frames: Left, Right, Disparity Map)
# ----------------------------------------------------------------------------------------------------------------------
canvas_w = 5 * 136 + 20
canvas_h = 3 * 136 + 60
canvas = Image.new('RGB', (canvas_w, canvas_h), (25, 28, 35))
cdraw = ImageDraw.Draw(canvas)

# Title
cdraw.text((20, 10), "Complex Multi-Object Stereo Vision: Left Frame, Right Frame, and 128x128 Disparity Map", fill=(255, 255, 255))
cdraw.text((20, 26), "Objects: [Left: Pedestrian (90 cm)] | [Center: Approaching Vehicle (90 -> 30 cm)] | [Right: Traffic Sign (75 cm)]", fill=(180, 210, 240))

row_labels = ["Left Camera", "Right Camera", "Disparity Map"]

for r, label in enumerate(row_labels):
    y_pos = 45 + r * 136
    for c in range(5):
        x_pos = 20 + c * 136
        if r == 0:
            tile = Image.fromarray(left_frames[c]).convert('RGB')
        elif r == 1:
            tile = Image.fromarray(right_frames[c]).convert('RGB')
        else:
            # Colorize disparity using colormap (Turbo/Inferno style)
            disp_gray = disp_frames[c]
            # Simple heat map: 0 = dark blue, 128 = green, 255 = yellow/red
            disp_rgb = np.zeros((H, W, 3), dtype=np.uint8)
            disp_rgb[:, :, 0] = np.clip(disp_gray * 2, 0, 255)
            disp_rgb[:, :, 1] = np.clip(255 - np.abs(disp_gray.astype(int) - 128) * 2, 0, 255)
            disp_rgb[:, :, 2] = np.clip((255 - disp_gray.astype(int)) * 2, 0, 255)
            # Mask out background
            mask_zero = (disp_gray == 0)
            disp_rgb[mask_zero] = [15, 20, 30]
            tile = Image.fromarray(disp_rgb)
            
        canvas.paste(tile, (x_pos, y_pos))
        if r == 0:
            cm_val = [90.0, 75.0, 64.3, 45.0, 30.0][c]
            cdraw.text((x_pos, y_pos - 12), f"Frame {c+1} ({cm_val} cm)", fill=(200, 230, 255))

artifact_path = "C:/Users/linga/.gemini/antigravity/brain/0291c975-da7d-45f2-a64c-10f2bf45788e/multi_object_stereo_preview.png"
canvas.save(artifact_path)
canvas.save("multi_object_stereo_preview.png")
print(f"Saved visual artifact to {artifact_path}")

