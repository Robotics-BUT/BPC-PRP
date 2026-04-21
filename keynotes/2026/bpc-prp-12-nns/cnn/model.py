"""Tiny CNN for 64x64 grayscale line-following images.

Kept deliberately small (~20k params) so it trains in seconds on a
laptop CPU and runs in real time on the robot's Raspberry Pi.
"""

from __future__ import annotations

import torch
import torch.nn as nn


class LineCNN(nn.Module):
    def __init__(self, num_classes: int = 3):
        super().__init__()
        self.features = nn.Sequential(
            # 1x64x64 -> 8x32x32
            nn.Conv2d(1, 8, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),
            # 8x32x32 -> 16x16x16
            nn.Conv2d(8, 16, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),
            # 16x16x16 -> 32x8x8
            nn.Conv2d(16, 32, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),
        )
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(32 * 8 * 8, 64),
            nn.ReLU(inplace=True),
            nn.Dropout(0.2),
            nn.Linear(64, num_classes),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.classifier(self.features(x))


if __name__ == "__main__":
    model = LineCNN()
    n_params = sum(p.numel() for p in model.parameters())
    print(f"Total parameters: {n_params:,}")
    # Smoke test: forward pass with a batch of 4.
    x = torch.zeros(4, 1, 64, 64)
    y = model(x)
    print("Input shape :", tuple(x.shape))
    print("Output shape:", tuple(y.shape))
