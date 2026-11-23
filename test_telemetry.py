from generate_complex_stereo_video import disp_frames
import numpy as np

for f_id in range(5):
    disp_vis = disp_frames[f_id]
    
    # Left Sector (Pedestrian): X in [20, 35], Y in [50, 80]
    v_l = int(disp_vis[50:80, 20:36].max())
    d_l = v_l * 15.0 / 255.0
    dist_l = 450.0 / d_l if d_l >= 4.0 else 999.0
    
    # Center Sector (Car): X in [45, 95], Y in [50, 95]
    v_c = int(disp_vis[50:95, 45:96].max())
    d_c = v_c * 15.0 / 255.0
    dist_c = 450.0 / d_c if d_c >= 4.0 else 999.0
    
    # Right Sector (Sign): X in [100, 124], Y in [45, 80]
    v_r = int(disp_vis[45:80, 100:125].max())
    d_r = v_r * 15.0 / 255.0
    dist_r = 450.0 / d_r if d_r >= 4.0 else 999.0
    
    cnt = (1 if dist_l < 200.0 else 0) + (1 if dist_c < 200.0 else 0) + (1 if dist_r < 200.0 else 0)
    
    led_state = ""
    if dist_c <= 35.0:
        led_state = "2 LEDs (Green+Red) SOLID ON"
    elif dist_c <= 55.0:
        led_state = "1 LED (Green) FAST BLINK (~7 Hz)"
    elif dist_c <= 70.0:
        led_state = "1 LED (Green) SOLID ON"
    else:
        led_state = "Both LEDs OFF"
        
    print(f"Frame {f_id+1}: Left={dist_l:5.1f} cm | Center={dist_c:5.1f} cm | Right={dist_r:5.1f} cm | Count={cnt} | LEDs: {led_state}")

