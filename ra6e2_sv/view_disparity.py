"""
Disparity Map Visualizer for RA6E2 Project (Supports both 64x64 and 128x128)
"""

import sys
import os
import numpy as np
from PIL import Image

def view_from_file(bin_path="disparity.bin"):
    if not os.path.exists(bin_path):
        print(f"File '{bin_path}' not found.")
        print("To export from e2 studio Memory view:")
        print("1. Enter address: &g_disparity_visual")
        print("2. Click Export -> Format: RAW Binary, Length: 16384 bytes (for 128x128) or 4096 (for 64x64)")
        print("3. Save as disparity.bin and run this script again.")
        return

    with open(bin_path, "rb") as f:
        raw_bytes = f.read()

    if len(raw_bytes) >= 128 * 128:
        dim = 128
    elif len(raw_bytes) >= 64 * 64:
        dim = 64
    else:
        print(f"Error: Unexpected byte length {len(raw_bytes)}")
        return

    arr = np.frombuffer(raw_bytes[:dim * dim], dtype=np.uint8).reshape((dim, dim))
    print(f"Loaded {bin_path} ({dim}x{dim}) successfully!")
    print(f"Min: {arr.min()}, Max: {arr.max()}")

    img = Image.fromarray(arr)
    img_preview = img.resize((384, 384), Image.Resampling.NEAREST)
    img_preview.show()
    img.save(f"disparity_{dim}x{dim}_out.png")
    print(f"Saved preview to disparity_{dim}x{dim}_out.png")

if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "disparity.bin"
    view_from_file(path)
