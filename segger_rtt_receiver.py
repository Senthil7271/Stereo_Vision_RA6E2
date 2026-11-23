"""
SEGGER J-Link RTT PC Receiver Script for Renesas RA6E2
Connects to J-Link RTT Telnet Port (localhost:19021) to:
  1. Print live text logs from the board.
  2. Send interactive commands ('1' for Image 0, '2' for Image 1, etc.).
  3. Detect framed binary packets (Magic 0xAA 0x55 0xDE 0xAD), verify CRC32,
     and save received 64x64 images as PNG and binary files.
"""

import socket
import struct
import zlib
import time
import os
import threading
from PIL import Image

HOST = '127.0.0.1'
PORT = 19021  # Default J-Link RTT Telnet port

MAGIC = b'\xAA\x55\xDE\xAD'
OUT_DIR = os.path.dirname(os.path.abspath(__file__))

def save_image(packet_id, payload):
    filename_bin = os.path.join(OUT_DIR, f"rtt_received_img_{packet_id}.bin")
    filename_png = os.path.join(OUT_DIR, f"rtt_received_img_{packet_id}.png")

    with open(filename_bin, "wb") as f:
        f.write(payload)

    if len(payload) == 4096:
        img = Image.frombytes('L', (64, 64), payload)
        img.save(filename_png)
        print(f"\n[SUCCESS] Image {packet_id} reconstructed & saved to:\n  -> {filename_png}\n  -> {filename_bin}")
    else:
        print(f"\n[INFO] Saved {len(payload)} bytes payload to {filename_bin}")

def receiver_loop(sock):
    buf = bytearray()
    print("[RECEIVER] Connected to J-Link RTT server. Listening for data...\n")

    while True:
        try:
            chunk = sock.recv(1024)
            if not chunk:
                print("\n[RECEIVER] Disconnected from J-Link RTT server.")
                break

            buf.extend(chunk)

            # Check for framed packets
            while True:
                magic_pos = buf.find(MAGIC)
                if magic_pos == -1:
                    # Print any text before discarding or wait
                    # Look for non-binary printable text
                    if len(buf) > 0:
                        text_part = bytes([b for b in buf if b in b'\r\n\t' or 32 <= b <= 126])
                        if text_part:
                            print(text_part.decode('ascii', errors='ignore'), end='', flush=True)
                        buf.clear()
                    break

                # If there is text before the magic header, print it
                if magic_pos > 0:
                    text_prefix = bytes([b for b in buf[:magic_pos] if b in b'\r\n\t' or 32 <= b <= 126])
                    if text_prefix:
                        print(text_prefix.decode('ascii', errors='ignore'), end='', flush=True)
                    del buf[:magic_pos]

                # We are at MAGIC (4 bytes)
                # Header needs at least 9 bytes: Magic (4) + ID (1) + Length (4)
                if len(buf) < 9:
                    break  # Wait for more data

                pkt_id = buf[4]
                payload_len = struct.unpack('<I', buf[5:9])[0]

                # Total packet size: 9 (header) + payload_len + 4 (crc32)
                total_pkt_size = 9 + payload_len + 4

                if len(buf) < total_pkt_size:
                    # Packet incomplete, wait for remaining bytes
                    break

                # Extract payload and CRC
                payload = bytes(buf[9:9 + payload_len])
                expected_crc = struct.unpack('<I', buf[9 + payload_len:total_pkt_size])[0]
                computed_crc = zlib.crc32(payload) & 0xFFFFFFFF

                print(f"\n[PACKET] Received Packet ID {pkt_id} ({payload_len} bytes)")
                if computed_crc == expected_crc:
                    print(f"[CRC OK] CRC32: 0x{computed_crc:08X} (Matches)")
                    save_image(pkt_id, payload)
                else:
                    print(f"[CRC MISMATCH] Computed: 0x{computed_crc:08X} != Expected: 0x{expected_crc:08X}")

                # Remove parsed packet from buffer
                del buf[:total_pkt_size]

        except Exception as e:
            print(f"\n[ERROR] Connection error: {e}")
            break

def main():
    print("=" * 60)
    print(" SEGGER J-Link RTT Python Receiver for Renesas RA6E2")
    print("=" * 60)
    print(f"Connecting to J-Link RTT on {HOST}:{PORT}...")
    print("Note: Ensure J-Link GDB Server or J-Link RTT Viewer is running.\n")

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((HOST, PORT))
    except Exception as e:
        print(f"[ERROR] Could not connect to {HOST}:{PORT}: {e}")
        print("Please start J-Link RTT Viewer or launch debug in e2 studio first.")
        return

    # Start background receiver thread
    t = threading.Thread(target=receiver_loop, args=(sock,), daemon=True)
    t.start()

    print("\n[READY] Type commands and press Enter:")
    print("  '1' -> Request & receive Image 0")
    print("  '2' -> Request & receive Image 1")
    print("  'h' -> View Hex dump")
    print("  'c' -> Run CRC integrity check on board")
    print("  't' -> Send sample telemetry")
    print("  'q' -> Quit\n")

    try:
        while True:
            cmd = input()
            if cmd.strip().lower() == 'q':
                break
            if cmd:
                sock.sendall((cmd + '\n').encode('ascii'))
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        print("\nReceiver closed.")

if __name__ == '__main__':
    main()
