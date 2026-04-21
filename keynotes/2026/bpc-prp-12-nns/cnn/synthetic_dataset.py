"""Synthetic line-following dataset.

Generates 64x64 grayscale images of a dark line on a light floor.
Class label is the action the robot should take. The part of the line
closest to the robot is at the TOP of the image (camera mounting):
    0 - turn LEFT   (line is in the left third, near the top)
    1 - go STRAIGHT (line is in the center third, near the top)
    2 - turn RIGHT  (line is in the right third, near the top)

We use this so the whole training pipeline runs offline on a laptop
in seconds. On the real robot the input comes from the RPi camera.
"""

from __future__ import annotations

import numpy as np
import torch
from torch.utils.data import Dataset


IMG_SIZE = 64


def _draw_line_image(
    rng: np.random.Generator,
    label: int,
) -> np.ndarray:
    """Draw one synthetic line image.

    The floor is bright (200..255). The line is dark (0..60). The line
    starts from the bottom at x = x_bottom, with a random slope so the
    picture is not boringly identical every time.
    """
    img = rng.integers(200, 256, size=(IMG_SIZE, IMG_SIZE), dtype=np.uint8)

    # Map class label to the bottom-x range of the line.
    third = IMG_SIZE // 3
    if label == 0:      # LEFT
        x_bottom = rng.integers(4, third)
    elif label == 1:    # STRAIGHT
        x_bottom = rng.integers(third, 2 * third)
    else:               # RIGHT
        x_bottom = rng.integers(2 * third, IMG_SIZE - 4)

    slope = rng.uniform(-0.6, 0.6)
    line_width = rng.integers(3, 6)
    line_value = rng.integers(0, 60)

    for y in range(IMG_SIZE):
        # y=IMG_SIZE-1 is the bottom row
        x = int(x_bottom + slope * (IMG_SIZE - 1 - y))
        for dx in range(-line_width // 2, line_width // 2 + 1):
            xi = x + dx
            if 0 <= xi < IMG_SIZE:
                img[y, xi] = line_value

    # Add a bit of Gaussian noise so the net has to learn something.
    noise = rng.normal(0, 8, img.shape)
    img = np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    # Camera sees the floor ahead, so what's close to the robot is at the
    # TOP of the image (near end of the line), not the bottom.
    img = np.flipud(img).copy()
    return img


class LineDataset(Dataset):
    def __init__(self, n_samples: int = 2000, seed: int = 0):
        self.n = n_samples
        self.rng = np.random.default_rng(seed)
        # Pre-generate to keep training fast and deterministic.
        self.images = np.empty((n_samples, IMG_SIZE, IMG_SIZE), dtype=np.uint8)
        self.labels = np.empty(n_samples, dtype=np.int64)
        for i in range(n_samples):
            label = int(self.rng.integers(0, 3))
            self.images[i] = _draw_line_image(self.rng, label)
            self.labels[i] = label

    def __len__(self) -> int:
        return self.n

    def __getitem__(self, idx: int):
        img = self.images[idx].astype(np.float32) / 255.0
        # Shape: (1, H, W) — one grayscale channel.
        x = torch.from_numpy(img).unsqueeze(0)
        y = torch.tensor(self.labels[idx], dtype=torch.long)
        return x, y


CLASS_NAMES = ["LEFT", "STRAIGHT", "RIGHT"]


if __name__ == "__main__":
    # Quick visual check: save a small grid of samples.
    import matplotlib.pyplot as plt

    ds = LineDataset(n_samples=9, seed=42)
    fig, axes = plt.subplots(3, 3, figsize=(6, 6))
    for ax, (img, label) in zip(axes.flat, ds):
        ax.imshow(img.squeeze().numpy(), cmap="gray", vmin=0, vmax=1)
        ax.set_title(CLASS_NAMES[int(label)])
        ax.axis("off")
    plt.tight_layout()
    plt.savefig("samples.png", dpi=100)
    print("Saved samples.png")
