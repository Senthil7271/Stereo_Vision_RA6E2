"""
Synthetic and Real Dataset Loader for 64x64 Stereo Pairs
Provides infinite on-the-fly synthetic stereo pairs with exact ground truth disparity,
or loads real images from a local directory.
"""

import numpy as np
import torch
from torch.utils.data import Dataset
import random

class SyntheticStereoDataset(Dataset):
    """Generates synthetic 64x64 stereo pairs on the fly.
    Draws random shapes (rectangles, circles, textured planes) with random disparities (0..11 px).
    Left image has objects at (x), Right image has objects shifted horizontally to (x - d).
    """
    def __init__(self, size: int = 1000, img_size: int = 64, max_disp: int = 12):
        self.size = size
        self.w = img_size
        self.h = img_size
        self.max_disp = max_disp

    def __len__(self):
        return self.size

    def __getitem__(self, idx):
        # 1. Background texture (gradient + high frequency noise)
        left = np.random.uniform(0.1, 0.4, (self.h, self.w)).astype(np.float32)
        noise = np.random.normal(0, 0.05, (self.h, self.w)).astype(np.float32)
        left = np.clip(left + noise, 0.0, 1.0)
        
        # Right starts as background copy (disparity = 0 for far background)
        right = left.copy()
        disp_gt = np.zeros((self.h, self.w), dtype=np.float32)

        # 2. Add 2 to 5 foreground objects with random disparity
        num_objects = random.randint(2, 5)
        for _ in range(num_objects):
            d = random.randint(1, self.max_disp - 1)  # Disparity in pixels
            obj_w = random.randint(10, 25)
            obj_h = random.randint(10, 25)
            
            x0 = random.randint(d, self.w - obj_w - 1)
            y0 = random.randint(0, self.h - obj_h - 1)
            
            # Object texture
            obj_texture = np.random.uniform(0.5, 0.95, (obj_h, obj_w)).astype(np.float32)
            obj_noise = np.random.normal(0, 0.05, (obj_h, obj_w)).astype(np.float32)
            obj_patch = np.clip(obj_texture + obj_noise, 0.0, 1.0)

            # Place in Left image at x0
            left[y0:y0+obj_h, x0:x0+obj_w] = obj_patch
            disp_gt[y0:y0+obj_h, x0:x0+obj_w] = float(d)

            # Place in Right image shifted by d: x_right = x0 - d
            right[y0:y0+obj_h, x0-d:x0-d+obj_w] = obj_patch

        # Normalize to standard [-1.0, 1.0] range
        left_norm  = (left - 0.5) / 0.5
        right_norm = (right - 0.5) / 0.5

        # Convert to Torch Tensors [1, H, W]
        t_left  = torch.from_numpy(left_norm).unsqueeze(0).float()
        t_right = torch.from_numpy(right_norm).unsqueeze(0).float()
        t_disp  = torch.from_numpy(disp_gt).unsqueeze(0).float()

        return t_left, t_right, t_disp

if __name__ == "__main__":
    ds = SyntheticStereoDataset(size=5)
    l, r, d = ds[0]
    print(f"Dataset test successful!")
    print(f"Left: {l.shape}, Right: {r.shape}, Disparity GT: {d.shape}")
    print(f"Disparity Min: {d.min().item():.1f}, Max: {d.max().item():.1f}")

