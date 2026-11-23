"""
Training script for MicroEdgeDepth (64x64 Stereo)
Optimized for fast CPU/GPU execution. Trains in ~2 minutes on PC.
"""

import os
import sys
import torch
import torch.nn as nn
from torch.utils.data import DataLoader
from tqdm import tqdm

from micro_model import MicroEdgeDepth, count_parameters
from dataset import SyntheticStereoDataset

def train(epochs: int = 15, batch_size: int = 32, lr: float = 1e-3):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"=======================================================")
    print(f" Training MicroEdgeDepth on {device}")
    print(f" Target Resolution : 64x64 Grayscale")
    print(f" Target Board      : Renesas FPB-RA6E2 (40 KB SRAM)")
    print(f"=======================================================")

    model = MicroEdgeDepth(num_disp=4, groups=4).to(device)
    print(f"Model Parameters: {count_parameters(model)}")

    # Datasets
    train_dataset = SyntheticStereoDataset(size=2000, img_size=64, max_disp=12)
    val_dataset   = SyntheticStereoDataset(size=200,  img_size=64, max_disp=12)

    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
    val_loader   = DataLoader(val_dataset, batch_size=batch_size, shuffle=False)

    optimizer = torch.optim.AdamW(model.parameters(), lr=lr, weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs)
    criterion = nn.SmoothL1Loss(beta=1.0)

    best_val_epe = float("inf")
    save_path = "micro_edgedepth_best.pth"

    for epoch in range(1, epochs + 1):
        model.train()
        train_loss = 0.0

        pbar = tqdm(train_loader, desc=f"Epoch {epoch:2d}/{epochs}")
        for left, right, disp_gt in pbar:
            left    = left.to(device)
            right   = right.to(device)
            disp_gt = disp_gt.to(device)

            optimizer.zero_grad()
            disp_pred = model(left, right)
            
            loss = criterion(disp_pred, disp_gt)
            loss.backward()
            optimizer.step()

            train_loss += loss.item()
            pbar.set_postfix({"loss": f"{loss.item():.4f}"})

        scheduler.step()
        train_loss /= len(train_loader)

        # Validation: End-Point Error (EPE in pixels)
        model.eval()
        val_epe = 0.0
        with torch.no_grad():
            for left, right, disp_gt in val_loader:
                left    = left.to(device)
                right   = right.to(device)
                disp_gt = disp_gt.to(device)

                disp_pred = model(left, right)
                # EPE: mean absolute pixel difference
                epe = torch.abs(disp_pred - disp_gt).mean()
                val_epe += epe.item()

        val_epe /= len(val_loader)
        print(f"Epoch {epoch:2d} | Train Loss: {train_loss:.4f} | Val EPE: {val_epe:.3f} px")

        if val_epe < best_val_epe:
            best_val_epe = val_epe
            torch.save({
                "epoch": epoch,
                "model_state_dict": model.state_dict(),
                "val_epe": val_epe,
            }, save_path)
            print(f"  --> Saved new best checkpoint to {save_path} (EPE: {val_epe:.3f} px)")

    print(f"\nTraining Complete! Best Validation EPE: {best_val_epe:.3f} pixels.")
    print(f"Model saved to: {os.path.abspath(save_path)}")

if __name__ == "__main__":
    train(epochs=10, batch_size=32, lr=1e-3)
