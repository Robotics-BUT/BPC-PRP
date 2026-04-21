"""Tiny CNN for 3x32x32 RGB images, 2-class output (cat vs. dog)."""

from __future__ import annotations

import torch
import torch.nn as nn


class DogCatCNN(nn.Module):
    def __init__(self, num_classes: int = 2):
        super().__init__()
        self.features = nn.Sequential(
            # 3x32x32 -> 16x16x16
            nn.Conv2d(3, 16, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),
            # 16x16x16 -> 32x8x8
            nn.Conv2d(16, 32, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),
            # 32x8x8 -> 64x4x4
            nn.Conv2d(32, 64, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),
        )
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(64 * 4 * 4, 128),
            nn.ReLU(inplace=True),
            nn.Dropout(0.2),
            nn.Linear(128, num_classes),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.classifier(self.features(x))


if __name__ == "__main__":
    m = DogCatCNN()
    n = sum(p.numel() for p in m.parameters())
    print(f"Total parameters: {n:,}")
    x = torch.zeros(2, 3, 32, 32)
    print("Output shape:", tuple(m(x).shape))
