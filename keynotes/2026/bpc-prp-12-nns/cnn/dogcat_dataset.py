"""Dog vs. Cat dataset built on top of CIFAR-10.

CIFAR-10 has 10 classes; we keep only cat (3) and dog (5) and remap to
{0: cat, 1: dog}. Images are 3x32x32 RGB.

The first call downloads CIFAR-10 (~170 MB) into ./data/ — afterwards
it is cached, so this only happens once per checkout.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import torch
import torchvision
from torch.utils.data import Dataset

HERE = Path(__file__).parent
DATA_ROOT = HERE / "data"

CLASS_NAMES = ["CAT", "DOG"]
CIFAR_CAT, CIFAR_DOG = 3, 5


def _filter_cat_dog(cifar: torchvision.datasets.CIFAR10):
    """Return (images uint8 NHWC, labels in {0,1}) for cat+dog only."""
    imgs = cifar.data  # (N, 32, 32, 3) uint8
    labels = np.array(cifar.targets)
    mask = (labels == CIFAR_CAT) | (labels == CIFAR_DOG)
    imgs = imgs[mask]
    labels = labels[mask]
    new_labels = np.where(labels == CIFAR_CAT, 0, 1).astype(np.int64)
    return imgs, new_labels


class DogCatDataset(Dataset):
    """Wraps a slice of the filtered CIFAR-10 cat+dog data.

    Use the `from_split` factory to get train / val / test splits with
    consistent indexing.
    """

    def __init__(self, images: np.ndarray, labels: np.ndarray):
        self.images = images
        self.labels = labels

    def __len__(self) -> int:
        return len(self.labels)

    def __getitem__(self, idx: int):
        img = self.images[idx].astype(np.float32) / 255.0  # H,W,C in [0,1]
        # PyTorch wants C,H,W
        x = torch.from_numpy(img).permute(2, 0, 1).contiguous()
        y = torch.tensor(int(self.labels[idx]), dtype=torch.long)
        return x, y


def build_splits(
    n_train: int,
    n_val: int,
    n_test: int,
    seed: int = 0,
) -> tuple[DogCatDataset, DogCatDataset, DogCatDataset]:
    """Download CIFAR-10 once and carve out three balanced splits."""
    DATA_ROOT.mkdir(exist_ok=True)
    train_full = torchvision.datasets.CIFAR10(
        root=str(DATA_ROOT), train=True, download=True
    )
    test_full = torchvision.datasets.CIFAR10(
        root=str(DATA_ROOT), train=False, download=True
    )
    tr_imgs, tr_labels = _filter_cat_dog(train_full)
    te_imgs, te_labels = _filter_cat_dog(test_full)

    rng = np.random.default_rng(seed)
    perm = rng.permutation(len(tr_imgs))
    tr_imgs, tr_labels = tr_imgs[perm], tr_labels[perm]

    n_train = min(n_train, len(tr_imgs) - n_val)
    n_val = min(n_val, len(tr_imgs) - n_train)
    n_test = min(n_test, len(te_imgs))

    train_ds = DogCatDataset(tr_imgs[:n_train], tr_labels[:n_train])
    val_ds = DogCatDataset(
        tr_imgs[n_train:n_train + n_val], tr_labels[n_train:n_train + n_val]
    )
    test_perm = rng.permutation(len(te_imgs))[:n_test]
    test_ds = DogCatDataset(te_imgs[test_perm], te_labels[test_perm])
    return train_ds, val_ds, test_ds


if __name__ == "__main__":
    tr, va, te = build_splits(2000, 400, 400, seed=0)
    print(f"train={len(tr)}  val={len(va)}  test={len(te)}")
    x, y = tr[0]
    print(f"sample: x.shape={tuple(x.shape)}  label={CLASS_NAMES[int(y)]}")
