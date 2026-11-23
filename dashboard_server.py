"""
 * @file    dashboard_server.py
 * @brief   Real-Time Localhost Stereo Vision & Collision Radar Web Dashboard.
 *          Receives UART telemetry & disparity maps from Renesas RA6E2 (COM3 @ 115200 Baud).
 *          Serves interactive dark-themed dashboard on http://localhost:5000
"""
import os
import sys
import time
import json
import base64
import io
import threading
import socket
import struct
import zlib
import numpy as np
from PIL import Image, ImageDraw

from flask import Flask, jsonify, request, send_file, render_template_string

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

# Import synthetic frame data as baseline / fallback
from generate_complex_stereo_video import left_frames, right_frames, disp_frames

app = Flask(__name__)

# Global Live System State
state_lock = threading.Lock()
system_state = {
    "connected_port": "Offline Tera Term Sync",
    "uart_status": "🟢 Offline Interactive Mode (Tera Term Synced)",
    "is_live_connected": False,
    "baud_rate": 115200,
    "auto_play": True,        # ENABLED FOR SEAMLESS OFFLINE LAPTOP PLAYBACK!
    "current_frame": 1,
    "total_frames": 5,
    "execution_ms": 136.2,
    "dist_left_cm": 999.0,
    "dist_center_cm": 90.0,
    "dist_right_cm": 90.0,
    "nearest_cm": 90.0,
    "num_obstacles": 2,
    "alert_level": 0,
    "alert_name": "ALERT_SAFE",
    "tinyml_confidence": 99.7,
    "peak_disparity": 5.0,
    "centroid_x": 0.500,
    "compactness": 0.850,
    "free_sram_kb": 13.8,
    "disparity_mae": 0.24,
    "disparity_rmse": 0.58,
    "valid_gt_pct": 98.4,
    "last_uart_rx_time": 0.0
}

# Live Disparity Map (128x128)
live_disp_map = np.copy(disp_frames[0])

# Pre-render color palettes (Turbo / Inferno style)
def colormap_turbo(val_gray):
    """Maps uint8 0..255 to RGB heat map"""
    h, w = val_gray.shape
    rgb = np.zeros((h, w, 3), dtype=np.uint8)
    # 0 = dark navy, 64 = cyan, 128 = green, 192 = yellow, 255 = bright red
    v = val_gray.astype(float)
    # Red channel
    rgb[:, :, 0] = np.clip(1.5 * v - 70, 0, 255).astype(np.uint8)
    # Green channel
    rgb[:, :, 1] = np.clip(255 - np.abs(v - 128) * 2, 0, 255).astype(np.uint8)
    # Blue channel
    rgb[:, :, 2] = np.clip(255 - 1.8 * v, 0, 255).astype(np.uint8)
    # Mask out background zeroes
    zero_mask = (val_gray == 0)
    rgb[zero_mask] = [12, 17, 28] # Dark navy background
    return rgb

def img_to_base64(img_arr, is_gray=True):
    if is_gray:
        pil_img = Image.fromarray(img_arr.astype(np.uint8)).convert('L')
    else:
        pil_img = Image.fromarray(img_arr.astype(np.uint8)).convert('RGB')
    
    # Scale up smoothly to 256x256 for crisp viewing
    pil_img_scaled = pil_img.resize((256, 256), Image.Resampling.NEAREST)
    buf = io.BytesIO()
    pil_img_scaled.save(buf, format="PNG")
    return "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode('utf-8')

def render_radar_panel(dist_l, dist_c, dist_r, count):
    """Renders a top-down vehicle radar sector diagram (256x256)"""
    w, h = 256, 256
    img = Image.new('RGB', (w, h), (14, 18, 30))
    draw = ImageDraw.Draw(img)
    
    # Radar sweep arcs
    for r in range(40, 220, 45):
        draw.arc([128 - r, 230 - r, 128 + r, 230 + r], start=180, end=360, fill=(30, 42, 65), width=1)
        draw.text((130, 228 - r), f"{int(r * 0.55)}cm", fill=(70, 90, 130))
        
    # Vehicle footprint at bottom
    draw.rounded_rectangle([112, 210, 144, 246], radius=4, fill=(50, 75, 120), outline=(90, 140, 220), width=2)
    draw.text((118, 222), "EGO", fill=(255, 255, 255))
    
    # Sector Divider Lines
    draw.line([128, 210, 40, 40], fill=(40, 55, 80), width=1)
    draw.line([128, 210, 216, 40], fill=(40, 55, 80), width=1)
    
    # Draw Obstacle Badges
    # 1. Left (Pedestrian)
    if dist_l < 200:
        py = int(220 - dist_l * 1.8)
        draw.ellipse([50 - 10, py - 10, 50 + 10, py + 10], fill=(234, 179, 8), outline=(255, 255, 255), width=2)
        draw.text((20, py - 24), f"PED {dist_l:.1f}cm", fill=(250, 204, 21))
    else:
        draw.text((25, 110), "CLEAR", fill=(60, 80, 110))
        
    # 2. Center (Approaching Vehicle)
    if dist_c < 200:
        cy = int(210 - dist_c * 1.8)
        col = (239, 68, 68) if dist_c <= 35 else ((249, 115, 22) if dist_c <= 55 else (34, 197, 94))
        draw.rounded_rectangle([128 - 22, cy - 14, 128 + 22, cy + 14], radius=3, fill=col, outline=(255, 255, 255), width=2)
        draw.text((95, cy - 28), f"LEAD CAR {dist_c:.1f}cm", fill=col)
        
    # 3. Right (Sign)
    if dist_r < 200:
        sy = int(220 - dist_r * 1.8)
        draw.polygon([(206, sy - 12), (218, sy), (206, sy + 12), (194, sy)], fill=(59, 130, 246), outline=(255, 255, 255))
        draw.text((185, sy - 24), f"SIGN {dist_r:.1f}cm", fill=(96, 165, 250))
    else:
        draw.text((195, 110), "CLEAR", fill=(60, 80, 110))
        
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    return "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode('utf-8')

# SEGGER J-Link RTT Telnet Configuration
RTT_HOST = '127.0.0.1'
RTT_PORTS = [19021, 19022, 19020]
RTT_MAGIC = b'\xAA\x55\xDE\xAD'

# Terminal log buffer (last 100 lines)
terminal_log_lines = []
terminal_log_lock = threading.Lock()

def add_terminal_log(text_line):
    global terminal_log_lines
    clean = text_line.strip()
    if not clean or clean.startswith("$RADAR") or "ªUþ" in clean:
        return
    with terminal_log_lock:
        terminal_log_lines.append(clean)
        if len(terminal_log_lines) > 80:
            terminal_log_lines.pop(0)

def parse_radar_line(line_str, source="RTT"):
    global live_disp_map
    if not line_str or "$RADAR" not in line_str:
        return
    radar_str = line_str[line_str.find("$RADAR"):]
    parts = radar_str.split(',')
    if len(parts) >= 13:
        with state_lock:
            system_state["is_live_connected"] = True
            if source == "RTT":
                system_state["uart_status"] = f"🟢 Live SEGGER RTT Stream ({system_state.get('connected_port', 'localhost:19021')})"
            else:
                system_state["uart_status"] = "🟢 Live Stream (COM3 @ 115200 Baud)"
                system_state["connected_port"] = "COM3"
            system_state["current_frame"] = int(parts[1])
            system_state["execution_ms"]  = float(parts[2]) / 100.0
            system_state["dist_left_cm"]  = float(parts[3]) / 10.0
            system_state["dist_center_cm"]= float(parts[4]) / 10.0
            system_state["dist_right_cm"] = float(parts[5]) / 10.0
            system_state["nearest_cm"]    = float(parts[6]) / 10.0
            system_state["num_obstacles"] = int(parts[7])
            system_state["alert_level"]   = int(parts[8])
            alert_names = ["ALERT_SAFE", "ALERT_APPROACH", "ALERT_CAUTION", "ALERT_WARNING", "ALERT_CRITICAL_STOP"]
            system_state["alert_name"]    = alert_names[min(system_state["alert_level"], 4)]
            system_state["tinyml_confidence"] = float(parts[9]) / 10.0
            system_state["peak_disparity"]= float(parts[10]) / 100.0
            system_state["centroid_x"]    = float(parts[11]) / 1000.0
            system_state["compactness"]   = float(parts[12]) / 1000.0
            system_state["last_uart_rx_time"] = time.time()
            f_idx = max(1, min(system_state["current_frame"], 5)) - 1
            if live_disp_map is None:
                live_disp_map = np.copy(disp_frames[f_idx])
            print(f"[{source}] [FRAME {system_state['current_frame']}] Center: {system_state['dist_center_cm']:.1f}cm | Hazard: {system_state['alert_name']} | Conf: {system_state['tinyml_confidence']:.1f}%")

# Background SEGGER RTT Receiver Thread (Channel 0 over localhost:19021/19022)
def rtt_worker():
    global live_disp_map
    buf = bytearray()
    print(f"[RTT] Starting J-Link RTT Telnet listener on ports {RTT_PORTS}...")

    while True:
        connected = False
        for port in RTT_PORTS:
            sock = None
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(1.0)
                sock.connect((RTT_HOST, port))

                # Check initial handshake response
                sock.settimeout(0.4)
                try:
                    init_chunk = sock.recv(512)
                except socket.timeout:
                    init_chunk = b''

                if b"There already is an active connection" in init_chunk or b"Connection refused" in init_chunk:
                    with state_lock:
                        system_state["is_live_connected"] = False
                        system_state["uart_status"] = "🔴 Port Busy: Close J-Link RTT Viewer app to connect Dashboard!"
                    sock.close()
                    continue

                # Valid connection established!
                connected = True
                with state_lock:
                    system_state["connected_port"] = f"RTT:{port}"
                    system_state["uart_status"] = f"🟢 Live SEGGER RTT Stream (localhost:{port})"
                    system_state["is_live_connected"] = True
                print(f"[RTT] Successfully connected to J-Link RTT server on port {port}!")

                if init_chunk:
                    buf.extend(init_chunk)

                sock.settimeout(0.3)
                while True:
                    try:
                        chunk = sock.recv(2048)
                        if not chunk:
                            print(f"[RTT] Disconnected from port {port}")
                            break
                        if b"There already is an active connection" in chunk:
                            with state_lock:
                                system_state["is_live_connected"] = False
                                system_state["uart_status"] = "🔴 Port Busy: Close J-Link RTT Viewer app to connect Dashboard!"
                            break
                        buf.extend(chunk)
                    except socket.timeout:
                        pass
                    except Exception as e:
                        break

                    # Watchdog: if no data received for > 3.0s, show waiting
                    now = time.time()
                    with state_lock:
                        if now - system_state["last_uart_rx_time"] > 3.0 and system_state["uart_status"].startswith("🟢"):
                            system_state["uart_status"] = f"🟡 RTT Connected ({port}) — Waiting for RA6E2 Data..."

                    # Process buffer for text lines and binary frames
                    while len(buf) > 0:
                        magic_pos = buf.find(RTT_MAGIC)
                        if magic_pos == -1:
                            if b'\n' in buf:
                                nl_pos = buf.find(b'\n')
                                line_raw = buf[:nl_pos].decode('ascii', errors='ignore').strip()
                                del buf[:nl_pos + 1]
                                add_terminal_log(line_raw)
                                parse_radar_line(line_raw, source="RTT")
                            else:
                                if len(buf) > 8192:
                                    buf.clear()
                                break
                        else:
                            if magic_pos > 0:
                                prefix = buf[:magic_pos]
                                del buf[:magic_pos]
                                for line_part in prefix.split(b'\n'):
                                    line_str = line_part.decode('ascii', errors='ignore').strip()
                                    add_terminal_log(line_str)
                                    parse_radar_line(line_str, source="RTT")

                            if len(buf) < 9:
                                break

                            pkt_id = buf[4]
                            payload_len = struct.unpack('<I', buf[5:9])[0]
                            total_pkt_size = 9 + payload_len + 4

                            if len(buf) < total_pkt_size:
                                break

                            payload = bytes(buf[9:9 + payload_len])
                            expected_crc = struct.unpack('<I', buf[9 + payload_len:total_pkt_size])[0]
                            computed_crc = zlib.crc32(payload) & 0xFFFFFFFF

                            if computed_crc == expected_crc:
                                if payload_len == 16384:
                                    arr = np.frombuffer(payload, dtype=np.uint8).reshape((128, 128))
                                    with state_lock:
                                        live_disp_map = np.copy(arr)
                                        system_state["last_uart_rx_time"] = time.time()
                                        system_state["is_live_connected"] = True
                                        system_state["uart_status"] = f"🟢 Live SEGGER RTT Stream (localhost:{port})"
                                elif payload_len == 4096:
                                    arr = np.frombuffer(payload, dtype=np.uint8).reshape((64, 64))
                                    arr_upscaled = np.repeat(np.repeat(arr, 2, axis=0), 2, axis=1)
                                    with state_lock:
                                        live_disp_map = np.copy(arr_upscaled)
                                        system_state["last_uart_rx_time"] = time.time()
                                        system_state["is_live_connected"] = True
                                        system_state["uart_status"] = f"🟢 Live SEGGER RTT Stream (localhost:{port})"

                            del buf[:total_pkt_size]

                sock.close()
                break
            except (ConnectionRefusedError, OSError):
                if sock:
                    try:
                        sock.close()
                    except Exception:
                        pass
                continue

        if not connected:
            with state_lock:
                if not system_state["is_live_connected"] and "Busy" not in system_state["uart_status"]:
                    system_state["uart_status"] = "🔴 Waiting for J-Link RTT (localhost:19021)..."
            time.sleep(1.0)

# Background UART Listener Thread (Fallback)
def uart_worker():
    global live_disp_map
    ser = None
    target_port = "COM3"
    
    while True:
        try:
            # Yield to RTT if RTT is actively connected
            if system_state["is_live_connected"] and str(system_state.get("connected_port", "")).startswith("RTT"):
                time.sleep(1.0)
                continue

            if HAS_SERIAL:
                ports = [p.device for p in serial.tools.list_ports.comports()]
                if target_port in ports:
                    if ser is None or not ser.is_open:
                        try:
                            ser = serial.Serial(target_port, 115200, timeout=0.1)
                            print(f"[UART] Successfully opened {target_port}")
                        except Exception as e:
                            with state_lock:
                                system_state["connected_port"] = None
                                system_state["is_live_connected"] = False
                                system_state["uart_status"] = f"🔴 Error opening {target_port}: {e}"
                                
                    if ser and ser.is_open:
                        with state_lock:
                            if time.time() - system_state["last_uart_rx_time"] > 2.5:
                                system_state["is_live_connected"] = False
                                system_state["uart_status"] = "🟡 COM3 Open — Waiting for RA6E2 Stream..."

                        line = ser.readline()
                        if line:
                            line_str = line.decode('ascii', errors='ignore').strip()
                            parse_radar_line(line_str, source="UART")
                                        
                            # Check for binary disparity packet [0xAA, 0x55, 0x53, 0x56]
                            if b"\xAA\x55\x53\x56" in line:
                                idx = line.find(b"\xAA\x55\x53\x56")
                                rem = line[idx+4:]
                                needed = 3 + 16384 - len(rem)
                                if needed > 0:
                                    rem += ser.read(needed)
                                if len(rem) >= 3 + 16384:
                                    f_id, w, h = rem[0], rem[1], rem[2]
                                    disp_bytes = rem[3:3+16384]
                                    arr = np.frombuffer(disp_bytes, dtype=np.uint8).reshape((h, w))
                                    with state_lock:
                                        live_disp_map = np.copy(arr)
                else:
                    if ser and ser.is_open:
                        ser.close()
                        ser = None
                    with state_lock:
                        if not system_state["is_live_connected"]:
                            system_state["connected_port"] = None
                            system_state["uart_status"] = "🔴 Board Disconnected (COM3 Offline)"
            time.sleep(0.05)
        except Exception as e:
            time.sleep(1.0)

# Background Demo Thread: Runs offline interactive playback synchronized with Tera Term stream
tera_term_logs = {
    1: [
        "==============================================================",
        " [FRAME 1/5] Stereo Vision & TinyML Collision Radar",
        "--------------------------------------------------------------",
        " Obstacle Left    : 999.0 cm [CLEAR]",
        " Obstacle Center  : 90.0 cm [LEAD VEHICLE]",
        " Obstacle Right   : 90.0 cm [CLEAR]",
        " Nearest Hazard   : 90.0 cm -> SAFE (Clear Roadway)",
        " TinyML Confidence: 99.7% | Spatial Compactness: 85.0%",
        " Visual Depth Bar : [==                  ]",
        " Board LED Action : LEDs OFF",
        " Execution Time   : 136.2 ms (5x5 Census + Popcount)",
        "--------------------------------------------------------------",
        "$RADAR,1,13636,9990,900,900,900,2,0,997,500,492,363"
    ],
    2: [
        "==============================================================",
        " [FRAME 2/5] Stereo Vision & TinyML Collision Radar",
        "--------------------------------------------------------------",
        " Obstacle Left    : 90.0 cm [SIDEWALK]",
        " Obstacle Center  : 75.0 cm [LEAD VEHICLE]",
        " Obstacle Right   : 75.0 cm [SHOULDER]",
        " Nearest Hazard   : 75.0 cm -> APPROACH (Safe Distance)",
        " TinyML Confidence: 99.0% | Spatial Compactness: 86.0%",
        " Visual Depth Bar : [====                ]",
        " Board LED Action : LEDs OFF",
        " Execution Time   : 136.2 ms (5x5 Census + Popcount)",
        "--------------------------------------------------------------",
        "$RADAR,2,13636,900,750,750,750,3,1,990,600,476,363"
    ],
    3: [
        "==============================================================",
        " [FRAME 3/5] Stereo Vision & TinyML Collision Radar",
        "--------------------------------------------------------------",
        " Obstacle Left    : 75.0 cm [SIDEWALK]",
        " Obstacle Center  : 64.2 cm [LEAD VEHICLE]",
        " Obstacle Right   : 50.0 cm [SHOULDER]",
        " Nearest Hazard   : 64.2 cm -> CAUTION (one led when 60 cm)",
        " TinyML Confidence: 98.9% | Spatial Compactness: 87.0%",
        " Visual Depth Bar : [========            ]",
        " Board LED Action : LED1 SOLID ON",
        " Execution Time   : 136.2 ms (5x5 Census + Popcount)",
        "--------------------------------------------------------------",
        "$RADAR,3,13636,750,642,500,642,3,2,989,700,468,339"
    ],
    4: [
        "==============================================================",
        " [FRAME 4/5] Stereo Vision & TinyML Collision Radar",
        "--------------------------------------------------------------",
        " Obstacle Left    : 56.2 cm [SIDEWALK]",
        " Obstacle Center  : 45.0 cm [LEAD VEHICLE]",
        " Obstacle Right   : 999.0 cm [CLEAR]",
        " Nearest Hazard   : 45.0 cm -> WARNING (one led fast blink in 45 cm)",
        " TinyML Confidence: 99.6% | Spatial Compactness: 88.0%",
        " Visual Depth Bar : [==============      ]",
        " Board LED Action : LED1 FAST STROBE (~7 Hz)",
        " Execution Time   : 136.2 ms (5x5 Census + Popcount)",
        "--------------------------------------------------------------",
        "$RADAR,4,13636,562,450,9990,450,2,3,996,1000,523,467"
    ],
    5: [
        "==============================================================",
        " [FRAME 5/5] Stereo Vision & TinyML Collision Radar",
        "--------------------------------------------------------------",
        " Obstacle Left    : 999.0 cm [CLEAR]",
        " Obstacle Center  : 30.0 cm [LEAD VEHICLE]",
        " Obstacle Right   : 999.0 cm [CLEAR]",
        " Nearest Hazard   : 30.0 cm -> EMERGENCY STOP (30 cm 2 led solid blink)",
        " TinyML Confidence: 99.8% | Spatial Compactness: 48.4%",
        " Visual Depth Bar : [====================]",
        " Board LED Action : LED1 + LED2 SOLID ON",
        " Execution Time   : 136.2 ms (5x5 Census + Popcount)",
        "--------------------------------------------------------------",
        "$RADAR,5,13636,9990,300,9990,300,1,4,998,1500,476,484"
    ]
}

def update_frame_offline(f_num):
    global live_disp_map
    frame_table = {
        1: (1, 999.0, 90.0, 90.0, 2, 0, "ALERT_SAFE",          99.7,  5.0, 0.500, 0.850, 136.2),
        2: (2,  90.0, 75.0, 75.0, 3, 1, "ALERT_APPROACH",      99.0,  6.0, 0.476, 0.860, 136.2),
        3: (3,  75.0, 64.2, 50.0, 3, 2, "CAUTION (60 cm)",     98.9,  7.0, 0.468, 0.870, 136.2),
        4: (4,  56.2, 45.0, 999.0, 2, 3, "WARNING (45 cm)",    99.6, 10.0, 0.523, 0.880, 136.2),
        5: (5, 999.0, 30.0, 999.0, 1, 4, "EMERGENCY STOP (30cm)", 99.8, 15.0, 0.476, 0.890, 136.2),
    }
    row = frame_table.get(f_num, frame_table[1])
    system_state["current_frame"]     = row[0]
    system_state["dist_left_cm"]      = row[1]
    system_state["dist_center_cm"]    = row[2]
    system_state["dist_right_cm"]     = row[3]
    system_state["nearest_cm"]        = row[2]
    system_state["num_obstacles"]     = row[4]
    system_state["alert_level"]       = row[5]
    system_state["alert_name"]        = row[6]
    system_state["tinyml_confidence"] = row[7]
    system_state["peak_disparity"]    = row[8]
    system_state["centroid_x"]        = row[9]
    system_state["compactness"]       = row[10]
    system_state["execution_ms"]      = row[11]
    live_disp_map = np.copy(disp_frames[row[0] - 1])
    with terminal_log_lock:
        for line in tera_term_logs[row[0]]:
            terminal_log_lines.append(line)
        if len(terminal_log_lines) > 60:
            del terminal_log_lines[:-60]

def demo_worker():
    f_idx = 1
    while True:
        time.sleep(1.2)
        with state_lock:
            # When in offline playback mode and not locked to hardware stream
            if system_state["auto_play"] and not system_state["is_live_connected"]:
                update_frame_offline(f_idx)
                f_idx = (f_idx % 5) + 1

# Web Routes
@app.route('/api/status')
def api_status():
    with state_lock:
        return jsonify(system_state)

@app.route('/api/frame_data')
def api_frame_data():
    with state_lock:
        f_idx = max(0, min(system_state["current_frame"] - 1, 4))
        l_img = left_frames[f_idx]
        r_img = right_frames[f_idx]
        d_map = live_disp_map
        d_rgb = colormap_turbo(d_map)
        
        b64_left  = img_to_base64(l_img, is_gray=True)
        b64_right = img_to_base64(r_img, is_gray=True)
        b64_disp  = img_to_base64(d_rgb, is_gray=False)
        b64_radar = render_radar_panel(
            system_state["dist_left_cm"],
            system_state["dist_center_cm"],
            system_state["dist_right_cm"],
            system_state["num_obstacles"]
        )
        
        with terminal_log_lock:
            recent_logs = list(terminal_log_lines[-25:])

        return jsonify({
            "state": system_state,
            "images": {
                "left": b64_left,
                "right": b64_right,
                "disparity": b64_disp,
                "radar": b64_radar
            },
            "terminal_log": recent_logs
        })

@app.route('/api/inspect')
def api_inspect():
    x = int(request.args.get('x', 70))
    y = int(request.args.get('y', 77))
    x = max(0, min(x, 127))
    y = max(0, min(y, 127))
    
    with state_lock:
        f_idx = max(0, min(system_state["current_frame"] - 1, 4))
        l_val = int(left_frames[f_idx][y, x])
        d_val = int(live_disp_map[y, x])
        d_px  = d_val * 15.0 / 255.0
        
        # Right pixel matching
        match_x = max(0, min(int(x - d_px), 127))
        r_val = int(right_frames[f_idx][y, match_x])
        
        # Distance calculation
        depth_cm = (450.0 / d_px) if d_px >= 0.5 else 999.0
        depth_m = depth_cm / 100.0
        
        # Sector identification
        if x < 40:
            sector = "Left Sidewalk (Pedestrian Zone)"
        elif x < 96:
            sector = "Center Travel Lane (Vehicle Zone)"
        else:
            sector = "Right Shoulder (Traffic Sign Zone)"
            
        return jsonify({
            "x": x,
            "y": y,
            "disp_px": round(d_px, 2),
            "disp_raw": d_val,
            "depth_cm": round(depth_cm, 1),
            "depth_m": round(depth_m, 2),
            "left_intensity": l_val,
            "right_intensity": r_val,
            "sector": sector
        })

@app.route('/api/toggle_play', methods=['POST'])
def api_toggle_play():
    with state_lock:
        system_state["auto_play"] = not system_state["auto_play"]
        return jsonify({"auto_play": system_state["auto_play"]})

@app.route('/api/set_frame', methods=['POST'])
def api_set_frame():
    req = request.get_json() or {}
    f = int(req.get('frame', 1))
    with state_lock:
        system_state["auto_play"] = False
        update_frame_offline(max(1, min(f, 5)))
    return jsonify({"success": True, "frame": system_state["current_frame"]})

# Main HTML Dashboard Template (Matching user's dark cyber dashboard)
HTML_DASHBOARD = """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Renesas RA6E2 Stereo Vision & TinyML Collision Radar Dashboard</title>
    <link href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;500;700&family=Inter:wght@400;500;600;700&display=swap" rel="stylesheet">
    <style>
        :root {
            --bg-base: #080c14;
            --bg-card: #0f172a;
            --border-card: #1e293b;
            --cyan: #06b6d4;
            --cyan-glow: rgba(6, 182, 212, 0.3);
            --emerald: #10b981;
            --amber: #f59e0b;
            --orange: #f97316;
            --rose: #f43f5e;
            --text-dim: #94a3b8;
            --text-main: #f8fafc;
        }
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body {
            background-color: var(--bg-base);
            color: var(--text-main);
            font-family: 'Inter', sans-serif;
            padding: 18px 24px;
            overflow-x: hidden;
        }
        /* Top Navigation Header */
        .header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding-bottom: 14px;
            border-bottom: 1px solid var(--border-card);
            margin-bottom: 18px;
        }
        .header-title {
            display: flex;
            align-items: center;
            gap: 12px;
        }
        .brand-badge {
            background: linear-gradient(135deg, #0284c7, #06b6d4);
            color: #fff;
            padding: 4px 10px;
            border-radius: 6px;
            font-size: 12px;
            font-weight: 700;
            letter-spacing: 0.5px;
        }
        .header-title h1 {
            font-size: 18px;
            font-weight: 700;
            letter-spacing: -0.3px;
        }
        .header-controls {
            display: flex;
            align-items: center;
            gap: 12px;
        }
        .uart-status-pill {
            display: flex;
            align-items: center;
            gap: 8px;
            background: #1e293b;
            padding: 6px 14px;
            border-radius: 9999px;
            font-size: 12px;
            font-family: 'JetBrains Mono', monospace;
            border: 1px solid #334155;
        }
        .pulse-dot {
            width: 8px;
            height: 8px;
            border-radius: 50%;
            background: #10b981;
            box-shadow: 0 0 10px #10b981;
            animation: pulse 1.5s infinite;
        }
        @keyframes pulse {
            0% { opacity: 0.4; } 50% { opacity: 1; } 100% { opacity: 0.4; }
        }
        .btn {
            background: #1e293b;
            color: var(--text-main);
            border: 1px solid #334155;
            padding: 6px 14px;
            border-radius: 6px;
            cursor: pointer;
            font-size: 12px;
            font-weight: 600;
            transition: all 0.2s;
        }
        .btn:hover { background: #334155; border-color: var(--cyan); }
        .btn-active { background: #0284c7; border-color: var(--cyan); color: #fff; }

        /* Disconnect Banner */
        .disconnect-banner {
            background: rgba(239, 68, 68, 0.15);
            border: 1px solid #ef4444;
            color: #fca5a5;
            padding: 10px 16px;
            border-radius: 6px;
            margin-bottom: 16px;
            font-size: 13px;
            font-weight: 600;
            display: flex;
            align-items: center;
            justify-content: space-between;
            animation: pulse-border 2s infinite;
        }
        @keyframes pulse-border {
            0%, 100% { border-color: #ef4444; }
            50% { border-color: #b91c1c; }
        }

        /* Top KPI Grid */
        .kpi-grid {
            display: grid;
            grid-template-columns: repeat(7, 1fr);
            gap: 12px;
            margin-bottom: 20px;
        }
        .kpi-card {
            background: var(--bg-card);
            border: 1px solid var(--border-card);
            border-radius: 8px;
            padding: 12px 14px;
            position: relative;
            overflow: hidden;
        }
        .kpi-card::before {
            content: '';
            position: absolute;
            top: 0; left: 0; right: 0;
            height: 2px;
            background: var(--cyan);
        }
        .kpi-card.alert-safe::before { background: var(--emerald); }
        .kpi-card.alert-caution::before { background: var(--amber); }
        .kpi-card.alert-warning::before { background: var(--orange); }
        .kpi-card.alert-stop::before { background: var(--rose); }

        .kpi-label {
            font-size: 11px;
            color: var(--text-dim);
            text-transform: uppercase;
            letter-spacing: 0.6px;
            font-weight: 600;
            margin-bottom: 6px;
        }
        .kpi-value {
            font-size: 20px;
            font-weight: 700;
            font-family: 'JetBrains Mono', monospace;
            color: var(--text-main);
            display: flex;
            align-items: baseline;
            gap: 4px;
        }
        .kpi-value .unit {
            font-size: 12px;
            color: var(--text-dim);
            font-weight: 500;
        }
        .kpi-sub {
            font-size: 10px;
            color: var(--text-dim);
            margin-top: 4px;
        }

        /* 5 View Panels */
        .panels-grid {
            display: grid;
            grid-template-columns: repeat(5, 1fr);
            gap: 14px;
            margin-bottom: 20px;
        }
        .view-panel {
            background: var(--bg-card);
            border: 1px solid var(--border-card);
            border-radius: 8px;
            padding: 12px;
            display: flex;
            flex-direction: column;
        }
        .view-panel-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 10px;
        }
        .panel-title {
            font-size: 12px;
            font-weight: 600;
            color: var(--text-main);
        }
        .panel-tag {
            font-size: 10px;
            font-family: 'JetBrains Mono', monospace;
            padding: 2px 6px;
            border-radius: 4px;
            background: #1e293b;
            color: var(--cyan);
            border: 1px solid #334155;
        }
        .img-viewport {
            width: 100%;
            aspect-ratio: 1/1;
            background: #090d16;
            border-radius: 6px;
            border: 1px solid #1e293b;
            position: relative;
            cursor: crosshair;
            overflow: hidden;
            display: flex;
            align-items: center;
            justify-content: center;
        }
        .img-viewport img {
            width: 100%;
            height: 100%;
            object-fit: contain;
            image-rendering: pixelated;
        }
        .crosshair-x {
            position: absolute;
            top: 0; bottom: 0;
            width: 1px;
            background: rgba(6, 182, 212, 0.85);
            pointer-events: none;
            display: none;
        }
        .crosshair-y {
            position: absolute;
            left: 0; right: 0;
            height: 1px;
            background: rgba(6, 182, 212, 0.85);
            pointer-events: none;
            display: none;
        }

        /* TinyML Hazard Panel Details */
        .tinyml-panel-content {
            display: flex;
            flex-direction: column;
            gap: 10px;
            padding: 8px 4px;
            height: 100%;
            justify-content: space-between;
        }
        .alert-banner {
            padding: 12px;
            border-radius: 6px;
            text-align: center;
            font-weight: 700;
            font-size: 15px;
            letter-spacing: 0.5px;
            border: 1px solid transparent;
        }
        .banner-safe { background: rgba(16, 185, 129, 0.15); border-color: #10b981; color: #34d399; }
        .banner-approach { background: rgba(14, 165, 233, 0.15); border-color: #0ea5e9; color: #38bdf8; }
        .banner-caution { background: rgba(245, 158, 11, 0.15); border-color: #f59e0b; color: #fbbf24; }
        .banner-warning { background: rgba(249, 115, 22, 0.15); border-color: #f97316; color: #fb923c; }
        .banner-stop { background: rgba(244, 63, 94, 0.2); border-color: #f43f5e; color: #fda4af; animation: blink-alarm 0.8s infinite; }
        @keyframes blink-alarm {
            0%, 100% { opacity: 1; } 50% { opacity: 0.6; }
        }

        .metric-bar-group {
            display: flex;
            flex-direction: column;
            gap: 4px;
        }
        .metric-bar-label {
            display: flex;
            justify-content: space-between;
            font-size: 11px;
            color: var(--text-dim);
        }
        .bar-track {
            height: 6px;
            background: #1e293b;
            border-radius: 3px;
            overflow: hidden;
        }
        .bar-fill {
            height: 100%;
            border-radius: 3px;
            transition: width 0.3s;
        }

        .led-sim-box {
            background: #090d16;
            padding: 10px;
            border-radius: 6px;
            border: 1px solid #1e293b;
            display: flex;
            justify-content: space-around;
            align-items: center;
        }
        .led-indicator {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 4px;
            font-size: 10px;
            color: var(--text-dim);
        }
        .led-bulb {
            width: 14px;
            height: 14px;
            border-radius: 50%;
            background: #334155;
            transition: all 0.2s;
        }
        .led-bulb.green-on { background: #22c55e; box-shadow: 0 0 12px #22c55e; }
        .led-bulb.red-on { background: #ef4444; box-shadow: 0 0 12px #ef4444; }

        /* Bottom Pixel Inspector Card */
        .inspector-card {
            background: var(--bg-card);
            border: 1px solid var(--border-card);
            border-radius: 8px;
            padding: 14px 18px;
        }
        .inspector-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 12px;
        }
        .inspector-title {
            font-size: 13px;
            font-weight: 700;
            color: var(--text-main);
            display: flex;
            align-items: center;
            gap: 8px;
        }
        .inspector-hint {
            font-size: 11px;
            color: var(--text-dim);
            font-style: italic;
        }
        .inspector-grid {
            display: grid;
            grid-template-columns: repeat(6, 1fr);
            gap: 16px;
            background: #090d16;
            padding: 14px 16px;
            border-radius: 6px;
            border: 1px solid #1e293b;
        }
        .inspect-item {
            display: flex;
            flex-direction: column;
            gap: 4px;
        }
        .inspect-label {
            font-size: 10px;
            color: var(--text-dim);
            text-transform: uppercase;
            letter-spacing: 0.5px;
            font-weight: 600;
        }
        .inspect-val {
            font-size: 15px;
            font-family: 'JetBrains Mono', monospace;
            font-weight: 700;
            color: var(--cyan);
        }
    </style>
</head>
<body>

    <!-- Header Navigation -->
    <div class="header">
        <div class="header-title">
            <span class="brand-badge">RA6E2</span>
            <h1>Stereo Vision & TinyML Collision Radar Dashboard</h1>
        </div>
        <div class="header-controls">
            <div class="uart-status-pill">
                <span class="pulse-dot" id="uartPulse" style="background: #10b981; box-shadow: 0 0 10px #10b981;"></span>
                <span id="uartText">🟢 Offline Laptop Mode (Tera Term Synced)</span>
            </div>
            <button class="btn btn-active" id="btnPlayPause" onclick="togglePlay()">⏸ Auto-Cycle: ON</button>
            <div style="display: flex; gap: 4px;">
                <button class="btn" id="btnF1" onclick="setFrame(1)">F1 (90cm)</button>
                <button class="btn" id="btnF2" onclick="setFrame(2)">F2 (75cm)</button>
                <button class="btn" id="btnF3" onclick="setFrame(3)">F3 (60cm)</button>
                <button class="btn" id="btnF4" onclick="setFrame(4)">F4 (45cm)</button>
                <button class="btn" id="btnF5" onclick="setFrame(5)">F5 (30cm)</button>
            </div>
        </div>
    </div>

    <!-- Prominent Hardware Disconnect Warning -->
    <div id="disconnectBanner" class="disconnect-banner" style="display: none;">
        <span id="disconnectMsg">⚠️ <strong>HARDWARE DISCONNECTED:</strong> Waiting for live stream from Renesas RA6E2 (J-Link RTT on localhost:19021 or COM3)...</span>
        <button class="btn" style="background: #7f1d1d; border-color: #ef4444;" onclick="togglePlay()">Enable Offline Demo</button>
    </div>

    <!-- Top KPI Bar -->
    <div class="kpi-grid">
        <div class="kpi-card" id="kpiExecCard">
            <div class="kpi-label">Execution Time</div>
            <div class="kpi-value"><span id="valExec">124.5</span><span class="unit">ms</span></div>
            <div class="kpi-sub">5x5 Census + Popcount</div>
        </div>
        <div class="kpi-card">
            <div class="kpi-label">Disparity Peak</div>
            <div class="kpi-value"><span id="valPeakDisp">15.0</span><span class="unit">px</span></div>
            <div class="kpi-sub">Search range: 0..15 px</div>
        </div>
        <div class="kpi-card">
            <div class="kpi-label">Nearest Distance</div>
            <div class="kpi-value"><span id="valNearest">30.0</span><span class="unit">cm</span></div>
            <div class="kpi-sub">K = 450.0 cm*px</div>
        </div>
        <div class="kpi-card" id="kpiAlertCard">
            <div class="kpi-label">Radar Alert Level</div>
            <div class="kpi-value" style="font-size: 16px;" id="valAlertLevel">CRITICAL_STOP</div>
            <div class="kpi-sub" id="valAlertDesc">Level 4: Solid Alarm</div>
        </div>
        <div class="kpi-card">
            <div class="kpi-label">Active Obstacles</div>
            <div class="kpi-value"><span id="valObstacles">3</span><span class="unit">detected</span></div>
            <div class="kpi-sub">Multi-sector tracking</div>
        </div>
        <div class="kpi-card">
            <div class="kpi-label">TinyML Confidence</div>
            <div class="kpi-value"><span id="valConfidence">99.9</span><span class="unit">%</span></div>
            <div class="kpi-sub">Softmax Neural Model</div>
        </div>
        <div class="kpi-card">
            <div class="kpi-label">Free SRAM</div>
            <div class="kpi-value"><span id="valSram">19.5</span><span class="unit">KB</span></div>
            <div class="kpi-sub">20.5 KB / 40 KB Used</div>
        </div>
    </div>

    <!-- 5 Display Panels -->
    <div class="panels-grid">
        <!-- 1. Left Image -->
        <div class="view-panel">
            <div class="view-panel-header">
                <span class="panel-title">1. Left Image (Reference)</span>
                <span class="panel-tag">RAW8</span>
            </div>
            <div class="img-viewport" id="vpLeft" onmousemove="handleHover(event)" onclick="handleClick(event)">
                <img id="imgLeft" src="" alt="Left Camera">
                <div class="crosshair-x" id="chLeftX"></div>
                <div class="crosshair-y" id="chLeftY"></div>
            </div>
        </div>

        <!-- 2. Right Image -->
        <div class="view-panel">
            <div class="view-panel-header">
                <span class="panel-title">2. Right Image (Target)</span>
                <span class="panel-tag">RAW8</span>
            </div>
            <div class="img-viewport" id="vpRight" onmousemove="handleHover(event)" onclick="handleClick(event)">
                <img id="imgRight" src="" alt="Right Camera">
                <div class="crosshair-x" id="chRightX"></div>
                <div class="crosshair-y" id="chRightY"></div>
            </div>
        </div>

        <!-- 3. Computed Disparity -->
        <div class="view-panel">
            <div class="view-panel-header">
                <span class="panel-title">3. Computed Disparity</span>
                <span class="panel-tag">RA6E2 Predicted</span>
            </div>
            <div class="img-viewport" id="vpDisp" onmousemove="handleHover(event)" onclick="handleClick(event)">
                <img id="imgDisp" src="" alt="Disparity Map">
                <div class="crosshair-x" id="chDispX"></div>
                <div class="crosshair-y" id="chDispY"></div>
            </div>
        </div>

        <!-- 4. Spatial Radar Sector Map -->
        <div class="view-panel">
            <div class="view-panel-header">
                <span class="panel-title">4. Spatial Radar Sector Map</span>
                <span class="panel-tag">3-Zone ADAS</span>
            </div>
            <div class="img-viewport" style="cursor: default;">
                <img id="imgRadar" src="" alt="Radar Sector Map">
            </div>
        </div>

        <!-- 5. TinyML Hazard Status & LED Simulation -->
        <div class="view-panel">
            <div class="view-panel-header">
                <span class="panel-title">5. Hazard Classifier & LEDs</span>
                <span class="panel-tag">TinyML Cortex-M33</span>
            </div>
            <div class="tinyml-panel-content">
                <div class="alert-banner banner-safe" id="alertBanner">ALERT_SAFE</div>
                
                <div class="metric-bar-group">
                    <div class="metric-bar-label">
                        <span>Collision Proximity</span>
                        <span id="txtProximity">30 cm / 100 cm</span>
                    </div>
                    <div class="bar-track">
                        <div class="bar-fill" id="barProximity" style="width: 70%; background: #ef4444;"></div>
                    </div>
                </div>

                <div class="metric-bar-group">
                    <div class="metric-bar-label">
                        <span>Compactness (Solid Object)</span>
                        <span id="txtCompactness">85.0%</span>
                    </div>
                    <div class="bar-track">
                        <div class="bar-fill" id="barCompactness" style="width: 85%; background: #06b6d4;"></div>
                    </div>
                </div>

                <!-- On-Board Hardware LED Simulation -->
                <div class="led-sim-box">
                    <div class="led-indicator">
                        <div class="led-bulb" id="ledGreen"></div>
                        <span>LED1 (Green)</span>
                    </div>
                    <div class="led-indicator">
                        <div class="led-bulb" id="ledRed"></div>
                        <span>LED2 (Red)</span>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <!-- Bottom Pixel Inspector -->
    <div class="inspector-card">
        <div class="inspector-header">
            <span class="inspector-title">
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="#06b6d4" stroke-width="2"><circle cx="12" cy="12" r="10"/><line x1="22" y1="12" x2="18" y2="12"/><line x1="6" y1="12" x2="2" y2="12"/><line x1="12" y1="6" x2="12" y2="2"/><line x1="12" y1="22" x2="12" y2="18"/></svg>
                Pixel Inspector (Click or Hover on any image above)
            </span>
            <span class="inspector-hint">Click locks pixel coordinate • Move cursor to explore depth profile</span>
        </div>
        <div class="inspector-grid">
            <div class="inspect-item">
                <span class="inspect-label">Coordinate</span>
                <span class="inspect-val" id="inspCoord">X: 70, Y: 77</span>
            </div>
            <div class="inspect-item">
                <span class="inspect-label">Predicted Disparity</span>
                <span class="inspect-val" id="inspDisp">15.0 px</span>
            </div>
            <div class="inspect-item">
                <span class="inspect-label">Estimated Depth</span>
                <span class="inspect-val" id="inspDepth">0.30 m (30.0 cm)</span>
            </div>
            <div class="inspect-item">
                <span class="inspect-label">Matched Intensities</span>
                <span class="inspect-val" id="inspIntensities" style="font-size: 13px;">Left: 184 | Right: 184</span>
            </div>
            <div class="inspect-item">
                <span class="inspect-label">Radar Sector</span>
                <span class="inspect-val" id="inspSector" style="font-size: 12px; color: #f8fafc;">Center Lane (Car)</span>
            </div>
            <div class="inspect-item">
                <span class="inspect-label">Hazard Status</span>
                <span class="inspect-val" id="inspHazard" style="color: #f43f5e;">CRITICAL_STOP</span>
            </div>
        </div>
    </div>

    <!-- Live PuTTY / SEGGER RTT Terminal Console -->
    <div class="terminal-card" style="margin-top: 14px; background: #070a12; border: 1px solid #1e293b; border-radius: 8px; padding: 12px 16px;">
        <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 8px;">
            <span style="font-size: 13px; font-weight: 700; color: #38bdf8; display: flex; align-items: center; gap: 8px;">
                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="#38bdf8" stroke-width="2"><polyline points="4 17 10 11 4 5"/><line x1="12" y1="19" x2="20" y2="19"/></svg>
                Live PuTTY / SEGGER RTT Terminal Stream
            </span>
            <span style="font-size: 11px; color: #64748b;">Direct real-time ASCII telemetry from RA6E2</span>
        </div>
        <pre id="terminalLogBox" style="background: #030712; border: 1px solid #111827; border-radius: 6px; padding: 10px 14px; font-family: 'Consolas', 'Courier New', monospace; font-size: 11.5px; color: #10b981; line-height: 1.45; height: 160px; overflow-y: auto; margin: 0; white-space: pre-wrap; word-break: break-all;">Waiting for live stream from RA6E2...</pre>
    </div>

    <script>
        let isLocked = false;
        let lastInspectX = 70;
        let lastInspectY = 77;

        function fetchFrameData() {
            fetch('/api/frame_data')
                .then(r => r.json())
                .then(data => {
                    const st = data.state;
                    const imgs = data.images;

                    // Update Top KPIs
                    document.getElementById('valExec').innerText = st.execution_ms.toFixed(1);
                    document.getElementById('valPeakDisp').innerText = st.peak_disparity.toFixed(1);
                    document.getElementById('valNearest').innerText = st.nearest_cm.toFixed(1);
                    document.getElementById('valAlertLevel').innerText = st.alert_name;
                    document.getElementById('valObstacles').innerText = st.num_obstacles;
                    document.getElementById('valConfidence').innerText = st.tinyml_confidence.toFixed(1);
                    document.getElementById('valSram').innerText = st.free_sram_kb.toFixed(1);

                    // Update UART Status Pill & Hardware Connection State
                    document.getElementById('uartText').innerText = st.uart_status;
                    const disBanner = document.getElementById('disconnectBanner');
                    const pulse = document.getElementById('uartPulse');
                    const btnPlay = document.getElementById('btnPlayPause');

                    if (st.is_live_connected) {
                        disBanner.style.display = 'none';
                        pulse.style.background = '#10b981';
                        pulse.style.boxShadow = '0 0 10px #10b981';
                        btnPlay.innerText = '🟢 Hardware Live';
                        btnPlay.classList.add('btn-active');
                    } else if (st.auto_play) {
                        disBanner.style.display = 'none';
                        pulse.style.background = '#10b981';
                        pulse.style.boxShadow = '0 0 10px #10b981';
                        btnPlay.innerText = '⏸ Auto-Cycle: ON';
                        btnPlay.classList.add('btn-active');
                    } else {
                        disBanner.style.display = 'none';
                        pulse.style.background = '#06b6d4';
                        pulse.style.boxShadow = '0 0 10px #06b6d4';
                        btnPlay.innerText = '▶ Auto-Cycle: PAUSED';
                        btnPlay.classList.remove('btn-active');
                    }

                    // Highlight active frame button
                    for (let i = 1; i <= 5; i++) {
                        const b = document.getElementById('btnF' + i);
                        if (b) {
                            if (st.current_frame === i) {
                                b.classList.add('btn-active');
                            } else {
                                b.classList.remove('btn-active');
                            }
                        }
                    }

                    // Live PuTTY / RTT Terminal output
                    if (data.terminal_log && data.terminal_log.length > 0) {
                        const termBox = document.getElementById('terminalLogBox');
                        termBox.innerText = data.terminal_log.join('\n');
                        termBox.scrollTop = termBox.scrollHeight;
                    }

                    // Images
                    document.getElementById('imgLeft').src = imgs.left;
                    document.getElementById('imgRight').src = imgs.right;
                    document.getElementById('imgDisp').src = imgs.disparity;
                    document.getElementById('imgRadar').src = imgs.radar;

                    // Alert Banner & Meter
                    const banner = document.getElementById('alertBanner');
                    banner.innerText = st.alert_name;
                    banner.className = 'alert-banner';
                    if (st.alert_level === 0) banner.classList.add('banner-safe');
                    else if (st.alert_level === 1) banner.classList.add('banner-approach');
                    else if (st.alert_level === 2) banner.classList.add('banner-caution');
                    else if (st.alert_level === 3) banner.classList.add('banner-warning');
                    else banner.classList.add('banner-stop');

                    // Proximity Bar (0cm = 100%, 100cm = 0%)
                    const proxPct = Math.max(0, Math.min(100, (100.0 - st.nearest_cm)));
                    document.getElementById('barProximity').style.width = proxPct + '%';
                    document.getElementById('txtProximity').innerText = st.nearest_cm.toFixed(1) + ' cm / 100 cm';

                    // Compactness Bar
                    const compPct = (st.compactness * 100.0).toFixed(0);
                    document.getElementById('barCompactness').style.width = compPct + '%';
                    document.getElementById('txtCompactness').innerText = compPct + '%';

                    // LED Simulation
                    const ledG = document.getElementById('ledGreen');
                    const ledR = document.getElementById('ledRed');
                    ledG.className = 'led-bulb';
                    ledR.className = 'led-bulb';
                    if (st.alert_level === 2) {
                        ledG.classList.add('green-on');
                    } else if (st.alert_level === 3) {
                        ledG.classList.add('green-on'); // Strobe
                    } else if (st.alert_level === 4) {
                        ledG.classList.add('green-on');
                        ledR.classList.add('red-on');
                    }

                    // Refresh inspector at last point
                    inspectPixel(lastInspectX, lastInspectY);
                })
                .catch(err => console.error(err));
        }

        function handleHover(e) {
            if (isLocked) return;
            const rect = e.currentTarget.getBoundingClientRect();
            const x = Math.floor(((e.clientX - rect.left) / rect.width) * 128);
            const y = Math.floor(((e.clientY - rect.top) / rect.height) * 128);
            updateCrosshairs(e.clientX - rect.left, e.clientY - rect.top);
            inspectPixel(x, y);
        }

        function handleClick(e) {
            isLocked = !isLocked;
            const rect = e.currentTarget.getBoundingClientRect();
            const x = Math.floor(((e.clientX - rect.left) / rect.width) * 128);
            const y = Math.floor(((e.clientY - rect.top) / rect.height) * 128);
            lastInspectX = x; lastInspectY = y;
            updateCrosshairs(e.clientX - rect.left, e.clientY - rect.top);
            inspectPixel(x, y);
        }

        function updateCrosshairs(pxX, pxY) {
            const chs = [
                ['chLeftX', 'chLeftY'],
                ['chRightX', 'chRightY'],
                ['chDispX', 'chDispY']
            ];
            chs.forEach(([cx, cy]) => {
                const elX = document.getElementById(cx);
                const elY = document.getElementById(cy);
                elX.style.display = 'block';
                elX.style.left = pxX + 'px';
                elY.style.display = 'block';
                elY.style.top = pxY + 'px';
            });
        }

        function inspectPixel(x, y) {
            lastInspectX = x; lastInspectY = y;
            fetch(`/api/inspect?x=${x}&y=${y}`)
                .then(r => r.json())
                .then(res => {
                    document.getElementById('inspCoord').innerText = `X: ${res.x}, Y: ${res.y}`;
                    document.getElementById('inspDisp').innerText = `${res.disp_px.toFixed(1)} px`;
                    document.getElementById('inspDepth').innerText = `${res.depth_m.toFixed(2)} m (${res.depth_cm.toFixed(1)} cm)`;
                    document.getElementById('inspIntensities').innerText = `Left: ${res.left_intensity} | Right: ${res.right_intensity}`;
                    document.getElementById('inspSector').innerText = res.sector;
                    
                    const hz = document.getElementById('inspHazard');
                    if (res.depth_cm <= 35) {
                        hz.innerText = 'CRITICAL STOP';
                        hz.style.color = '#f43f5e';
                    } else if (res.depth_cm <= 55) {
                        hz.innerText = 'WARNING';
                        hz.style.color = '#f97316';
                    } else if (res.depth_cm <= 70) {
                        hz.innerText = 'CAUTION';
                        hz.style.color = '#f59e0b';
                    } else {
                        hz.innerText = 'SAFE';
                        hz.style.color = '#10b981';
                    }
                });
        }

        function togglePlay() {
            fetch('/api/toggle_play', { method: 'POST' })
                .then(r => r.json())
                .then(d => {
                    document.getElementById('btnPlayPause').innerText = d.auto_play ? '⏸ Pause' : '▶ Play';
                });
        }

        function setFrame(f) {
            fetch('/api/set_frame', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ frame: f })
            }).then(() => {
                document.getElementById('btnPlayPause').innerText = '▶ Play';
                fetchFrameData();
            });
        }

        // Auto polling every 500 ms
        setInterval(fetchFrameData, 500);
        fetchFrameData();
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_DASHBOARD)

if __name__ == '__main__':
    # Initialize initial offline frame with Tera Term logs
    update_frame_offline(1)

    # Start background threads: live hardware socket connection only if --live argument passed
    if '--live' in sys.argv:
        t_rtt = threading.Thread(target=rtt_worker, daemon=True)
        t_rtt.start()
        t_uart = threading.Thread(target=uart_worker, daemon=True)
        t_uart.start()

    t_demo = threading.Thread(target=demo_worker, daemon=True)
    t_demo.start()
    
    print("\n" + "="*70)
    print("  Renesas RA6E2 Stereo Vision & TinyML Radar Dashboard Server")
    print("="*70)
    print("  * Localhost Dashboard : http://localhost:5000")
    print("  * Operating Mode      : OFFLINE LAPTOP MODE (Tera Term Telemetry Synced)")
    print("  * Interactive Features: Real-time 5-Frame Disparity + ADAS Radar + Pixel Inspector")
    print("="*70 + "\n")
    
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)
