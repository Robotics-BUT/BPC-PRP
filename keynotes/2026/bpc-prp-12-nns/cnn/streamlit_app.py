"""Streamlit demo: train & evaluate a tiny CNN on one of two tasks.

Run with:
    streamlit run streamlit_app.py

Pick the task from the sidebar:
  • Line follower — 64×64 grayscale synthetic line images, 3 classes.
  • Cat vs Dog    — CIFAR-10 cat/dog subset, 32×32 RGB, 2 classes.

Tabs:
  📊 Dataset       - peek at train/val/test splits
  🏗 Model         - architecture
  🏋 Training      - live loss/accuracy curves
  ✅ Evaluation     - confusion matrix + sample predictions
  🔬 Features      - learned filters & per-layer activations on one image
  📦 ONNX          - export + test-set evaluation via ONNX Runtime
  🔌 ONNX Runtime  - run any .onnx on a single image (test sample or upload)
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import streamlit as st
import torch
import torch.nn as nn
from torch.utils.data import DataLoader

from dogcat_dataset import (
    CLASS_NAMES as DOGCAT_CLASSES,
    DogCatDataset,
    build_splits as _build_dogcat,
)
from dogcat_model import DogCatCNN
from model import LineCNN
from synthetic_dataset import CLASS_NAMES as LINE_CLASSES, LineDataset


HERE = Path(__file__).parent


def pick_device() -> torch.device:
    if torch.backends.mps.is_available():
        return torch.device("mps")
    if torch.cuda.is_available():
        return torch.device("cuda")
    return torch.device("cpu")


@st.cache_resource(show_spinner="Generating synthetic line datasets…")
def build_line_splits(n_train: int, n_val: int, n_test: int, seed: int):
    return (
        LineDataset(n_train, seed=seed),
        LineDataset(n_val, seed=seed + 1),
        LineDataset(n_test, seed=seed + 2),
    )


@st.cache_resource(show_spinner="Loading CIFAR-10 cat+dog (downloads ~170 MB once)…")
def build_dogcat_splits(n_train: int, n_val: int, n_test: int, seed: int):
    return _build_dogcat(n_train, n_val, n_test, seed)


# ────────────────────────────────────────────────────────────────────────────
# Per-task config
# ────────────────────────────────────────────────────────────────────────────

@dataclass
class TaskConfig:
    key: str
    model_cls: type
    class_names: list[str]
    build_splits: Callable
    input_shape: tuple  # (C, H, W) per single sample
    is_rgb: bool
    onnx_name: str
    dataset_ranges: dict  # {"train":(min,max,default,step), ...}
    upload_pil_mode: str  # "L" or "RGB"
    upload_size: tuple    # (H, W)
    model_desc: str


TASKS: dict[str, TaskConfig] = {
    "Line follower (synthetic)": TaskConfig(
        key="line",
        model_cls=LineCNN,
        class_names=LINE_CLASSES,
        build_splits=build_line_splits,
        input_shape=(1, 64, 64),
        is_rgb=False,
        onnx_name="line_cnn.onnx",
        dataset_ranges={
            "train": (200, 8000, 4000, 200),
            "val":   (100, 2000,  800, 100),
            "test":  (100, 2000,  800, 100),
        },
        upload_pil_mode="L",
        upload_size=(64, 64),
        model_desc=(
            "Input is a `1×64×64` grayscale image. Three conv→ReLU→maxpool "
            "blocks shrink it to `32×8×8`, then a small MLP outputs 3 class "
            "logits."
        ),
    ),
    "Cat vs Dog (CIFAR-10)": TaskConfig(
        key="dogcat",
        model_cls=DogCatCNN,
        class_names=DOGCAT_CLASSES,
        build_splits=build_dogcat_splits,
        input_shape=(3, 32, 32),
        is_rgb=True,
        onnx_name="dogcat_cnn.onnx",
        dataset_ranges={
            "train": (500, 10000, 6000, 500),
            "val":   (200,  2000, 1000, 100),
            "test":  (200,  2000, 1000, 100),
        },
        upload_pil_mode="RGB",
        upload_size=(32, 32),
        model_desc=(
            "Input is a `3×32×32` RGB image. Three conv→ReLU→maxpool blocks "
            "shrink it to `64×4×4`, then a small MLP outputs 2 class logits."
        ),
    ),
}


def to_display(img: torch.Tensor, cfg: TaskConfig) -> np.ndarray:
    if cfg.is_rgb:
        return img.permute(1, 2, 0).numpy()
    return img.squeeze().numpy()


def imshow_sample(ax, img: torch.Tensor, cfg: TaskConfig) -> None:
    arr = to_display(img, cfg)
    if cfg.is_rgb:
        ax.imshow(arr)
    else:
        ax.imshow(arr, cmap="gray", vmin=0, vmax=1)


def sample_grid(ds, n: int, seed: int, cfg: TaskConfig):
    rng = np.random.default_rng(seed)
    idxs = rng.choice(len(ds), size=n, replace=False)
    cols = int(np.ceil(np.sqrt(n)))
    rows = int(np.ceil(n / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(cols * 1.4, rows * 1.4))
    for ax, idx in zip(np.array(axes).flat, idxs):
        img, label = ds[int(idx)]
        imshow_sample(ax, img, cfg)
        ax.set_title(cfg.class_names[int(label)], fontsize=8)
        ax.axis("off")
    for ax in np.array(axes).flat[len(idxs):]:
        ax.axis("off")
    fig.tight_layout()
    return fig


def class_distribution(ds, cfg: TaskConfig) -> pd.DataFrame:
    labels = np.array(ds.labels)
    counts = [int((labels == i).sum()) for i in range(len(cfg.class_names))]
    return pd.DataFrame(
        {"class": cfg.class_names, "count": counts}
    ).set_index("class")


def accuracy(logits: torch.Tensor, targets: torch.Tensor) -> float:
    return (logits.argmax(dim=1) == targets).float().mean().item()


def evaluate(model: nn.Module, loader: DataLoader, device: torch.device):
    model.eval()
    all_preds, all_targets = [], []
    with torch.no_grad():
        for x, y in loader:
            x = x.to(device)
            preds = model(x).argmax(dim=1).cpu()
            all_preds.append(preds)
            all_targets.append(y)
    preds = torch.cat(all_preds).numpy()
    targets = torch.cat(all_targets).numpy()
    return preds, targets, float((preds == targets).mean())


def confusion_matrix_fig(preds: np.ndarray, targets: np.ndarray, cfg: TaskConfig):
    n = len(cfg.class_names)
    cm = np.zeros((n, n), dtype=int)
    for t, p in zip(targets, preds):
        cm[int(t), int(p)] += 1
    fig, ax = plt.subplots(figsize=(4, 4))
    im = ax.imshow(cm, cmap="Blues")
    ax.set_xticks(range(n), cfg.class_names)
    ax.set_yticks(range(n), cfg.class_names)
    ax.set_xlabel("predicted")
    ax.set_ylabel("true")
    for i in range(n):
        for j in range(n):
            color = "white" if cm[i, j] > cm.max() / 2 else "black"
            ax.text(j, i, str(cm[i, j]), ha="center", va="center", color=color)
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    fig.tight_layout()
    return fig


def model_summary(model: nn.Module) -> str:
    lines = [str(model), ""]
    total = sum(p.numel() for p in model.parameters())
    trainable = sum(p.numel() for p in model.parameters() if p.requires_grad)
    lines.append(f"Total parameters    : {total:,}")
    lines.append(f"Trainable parameters: {trainable:,}")
    return "\n".join(lines)


def tile_maps(maps: np.ndarray) -> np.ndarray:
    C, H, W = maps.shape
    cols = int(np.ceil(np.sqrt(C)))
    rows = int(np.ceil(C / cols))
    grid = np.zeros((rows * H, cols * W), dtype=np.float32)
    for i in range(C):
        r, c = divmod(i, cols)
        m = maps[i].astype(np.float32)
        mn, mx = float(m.min()), float(m.max())
        if mx > mn:
            m = (m - mn) / (mx - mn)
        grid[r * H:(r + 1) * H, c * W:(c + 1) * W] = m
    return grid


def collect_activations(model: nn.Module, x: torch.Tensor):
    acts: list[tuple[str, torch.Tensor]] = []
    block = 0
    sublayer = ""
    with torch.no_grad():
        h = x
        for layer in model.features:
            h = layer(h)
            kind = type(layer).__name__
            if kind == "Conv2d":
                block += 1
                sublayer = "conv"
            elif kind == "ReLU":
                sublayer = "relu"
            elif kind == "MaxPool2d":
                sublayer = "pool"
            acts.append((f"block {block} · {sublayer}", h.clone()))
    return acts


def first_layer_filters_fig(model: nn.Module):
    w = model.features[0].weight.detach().cpu().numpy()  # (N, in_c, 3, 3)
    n_f, in_c = w.shape[:2]
    if in_c == 3:
        fig, axes = plt.subplots(2, n_f // 2, figsize=(n_f // 2 * 1.0, 2.4))
        for i, ax in enumerate(np.array(axes).flat):
            k = w[i].transpose(1, 2, 0)
            mn, mx = k.min(), k.max()
            if mx > mn:
                k = (k - mn) / (mx - mn)
            ax.imshow(k)
            ax.set_title(f"f{i}", fontsize=7)
            ax.axis("off")
    else:
        fig, axes = plt.subplots(1, n_f, figsize=(n_f * 1.0, 1.2))
        for i, ax in enumerate(np.array(axes).flat):
            k = w[i, 0]
            mn, mx = k.min(), k.max()
            if mx > mn:
                k = (k - mn) / (mx - mn)
            ax.imshow(k, cmap="gray", vmin=0, vmax=1)
            ax.set_title(f"f{i}", fontsize=7)
            ax.axis("off")
    fig.tight_layout()
    return fig


# ────────────────────────────────────────────────────────────────────────────
# UI
# ────────────────────────────────────────────────────────────────────────────

st.set_page_config(page_title="Tiny CNN demo", layout="wide")

with st.sidebar:
    st.header("Task")
    task_name = st.selectbox(
        "Dataset / model",
        list(TASKS.keys()),
        index=0,
        help="Switching tasks keeps each model's trained state separately.",
    )
    cfg = TASKS[task_name]

    st.header("Dataset")
    tr_min, tr_max, tr_def, tr_step = cfg.dataset_ranges["train"]
    va_min, va_max, va_def, va_step = cfg.dataset_ranges["val"]
    te_min, te_max, te_def, te_step = cfg.dataset_ranges["test"]
    n_train = st.slider("Train samples", tr_min, tr_max, tr_def, step=tr_step)
    n_val = st.slider("Validation samples", va_min, va_max, va_def, step=va_step)
    n_test = st.slider("Test samples", te_min, te_max, te_def, step=te_step)
    seed = st.number_input("Seed", 0, 9999, 0)

    st.header("Training")
    epochs = st.slider("Epochs", 1, 30, 8)
    batch_size = st.select_slider(
        "Batch size", options=[16, 32, 64, 128, 256], value=64
    )
    lr = st.select_slider(
        "Learning rate",
        options=[1e-4, 3e-4, 1e-3, 3e-3, 1e-2],
        value=1e-3,
        format_func=lambda v: f"{float(v):.0e}",
    )

st.title("🧠 Tiny CNN — train & evaluate")
st.caption(
    f"Current task: **{task_name}** — switch in the sidebar. "
    "Walks through dataset, model, training, evaluation, features, and ONNX."
)

CLASS_NAMES = cfg.class_names
ONNX_PATH = HERE / cfg.onnx_name
ONNX_INPUT_SHAPE = (1, *cfg.input_shape)


def skey(name: str) -> str:
    return f"{cfg.key}_{name}"


for k in ("metrics", "model_state"):
    st.session_state.setdefault(skey(k), None)


train_ds, val_ds, test_ds = cfg.build_splits(n_train, n_val, n_test, seed)

(tab_data, tab_model, tab_train, tab_eval, tab_features,
 tab_onnx, tab_runtime) = st.tabs(
    ["📊 Dataset", "🏗 Model", "🏋 Training", "✅ Evaluation",
     "🔬 Features", "📦 ONNX", "🔌 ONNX Runtime"]
)

# ── Dataset ────────────────────────────────────────────────────────────────
with tab_data:
    st.subheader("Splits")
    cols = st.columns(3)
    for col, name, ds in zip(
        cols, ["Train", "Validation", "Test"], [train_ds, val_ds, test_ds]
    ):
        with col:
            st.markdown(f"**{name}** — {len(ds)} samples")
            st.pyplot(sample_grid(ds, 9, seed, cfg), width="stretch")
            st.bar_chart(class_distribution(ds, cfg))

# ── Model ──────────────────────────────────────────────────────────────────
with tab_model:
    st.subheader("Architecture")
    st.code(model_summary(cfg.model_cls()), language="text")
    st.markdown(cfg.model_desc)
    if cfg.key == "dogcat":
        st.warning(
            "CIFAR-10 cat-vs-dog is genuinely hard — a tiny CNN like this "
            "will plateau around **70–75 %** test accuracy. That's expected: "
            "a great demo of where simple CNNs run out of steam."
        )

# ── Training ───────────────────────────────────────────────────────────────
with tab_train:
    st.subheader("Train the model")
    device = pick_device()
    st.caption(f"Device: `{device}`")

    train_btn = st.button("▶ Start training", type="primary", key=skey("train_btn"))
    chart_slot = st.empty()
    progress_slot = st.empty()
    log_slot = st.empty()

    if train_btn:
        train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True)
        val_loader = DataLoader(val_ds, batch_size=batch_size)

        model = cfg.model_cls().to(device)
        optim = torch.optim.Adam(model.parameters(), lr=lr)
        loss_fn = nn.CrossEntropyLoss()

        metrics = {"epoch": [], "train_loss": [], "train_acc": [], "val_acc": []}
        log_lines = []
        progress = progress_slot.progress(0.0, text="Starting…")

        for epoch in range(1, epochs + 1):
            t0 = time.time()
            model.train()
            running_loss = running_acc = 0.0
            n_batches = 0
            for x, y in train_loader:
                x, y = x.to(device), y.to(device)
                optim.zero_grad()
                logits = model(x)
                loss = loss_fn(logits, y)
                loss.backward()
                optim.step()
                running_loss += loss.item()
                running_acc += accuracy(logits, y)
                n_batches += 1

            model.eval()
            val_acc, val_n = 0.0, 0
            with torch.no_grad():
                for x, y in val_loader:
                    x, y = x.to(device), y.to(device)
                    val_acc += accuracy(model(x), y)
                    val_n += 1

            tr_loss = running_loss / n_batches
            tr_acc = running_acc / n_batches
            va_acc = val_acc / val_n
            dt = time.time() - t0

            metrics["epoch"].append(epoch)
            metrics["train_loss"].append(tr_loss)
            metrics["train_acc"].append(tr_acc)
            metrics["val_acc"].append(va_acc)
            log_lines.append(
                f"epoch {epoch:02d} | loss {tr_loss:.4f} | "
                f"train_acc {tr_acc:.3f} | val_acc {va_acc:.3f} | {dt:.1f}s"
            )

            df = pd.DataFrame(metrics).set_index("epoch")
            with chart_slot.container():
                c1, c2 = st.columns(2)
                c1.markdown("**Loss**")
                c1.line_chart(df[["train_loss"]])
                c2.markdown("**Accuracy**")
                c2.line_chart(df[["train_acc", "val_acc"]])
            progress.progress(epoch / epochs, text=f"Epoch {epoch}/{epochs}")
            log_slot.code("\n".join(log_lines), language="text")

        st.session_state[skey("metrics")] = metrics
        st.session_state[skey("model_state")] = {
            k: v.cpu() for k, v in model.state_dict().items()
        }
        st.success("Training finished. Switch to the Evaluation tab.")
    elif st.session_state[skey("metrics")] is not None:
        df = pd.DataFrame(st.session_state[skey("metrics")]).set_index("epoch")
        with chart_slot.container():
            c1, c2 = st.columns(2)
            c1.markdown("**Loss**")
            c1.line_chart(df[["train_loss"]])
            c2.markdown("**Accuracy**")
            c2.line_chart(df[["train_acc", "val_acc"]])

# ── Evaluation ─────────────────────────────────────────────────────────────
with tab_eval:
    st.subheader("Test-set evaluation")
    if st.session_state[skey("model_state")] is None:
        st.info("Train a model first (Training tab).")
    else:
        device = pick_device()
        model = cfg.model_cls().to(device)
        model.load_state_dict(st.session_state[skey("model_state")])

        test_loader = DataLoader(test_ds, batch_size=batch_size)
        preds, targets, acc = evaluate(model, test_loader, device)

        c1, c2 = st.columns([1, 1])
        with c1:
            st.metric("Test accuracy", f"{acc * 100:.2f} %")
            for cls_idx, cls_name in enumerate(CLASS_NAMES):
                mask = targets == cls_idx
                if mask.any():
                    cls_acc = (preds[mask] == cls_idx).mean()
                    st.write(f"• **{cls_name}**: {cls_acc * 100:.1f} %")
        with c2:
            st.pyplot(confusion_matrix_fig(preds, targets, cfg), width="stretch")

        st.markdown("---")
        st.subheader("Sample predictions")
        only_wrong = st.checkbox(
            "Show only mistakes", value=False, key=skey("only_wrong")
        )
        candidates = (
            np.where(preds != targets)[0] if only_wrong else np.arange(len(test_ds))
        )
        if len(candidates) == 0:
            st.success("No mistakes on the test set. 🎉")
        else:
            n_show = min(12, len(candidates))
            rng = np.random.default_rng(seed + 99)
            picked = rng.choice(candidates, size=n_show, replace=False)
            cols = st.columns(4)
            for i, idx in enumerate(picked):
                img, label = test_ds[int(idx)]
                pred = int(preds[idx])
                ok = pred == int(label)
                with cols[i % 4]:
                    fig, ax = plt.subplots(figsize=(2, 2))
                    imshow_sample(ax, img, cfg)
                    ax.set_title(
                        f"{'✓' if ok else '✗'} pred={CLASS_NAMES[pred]}\n"
                        f"true={CLASS_NAMES[int(label)]}",
                        fontsize=8,
                        color="green" if ok else "red",
                    )
                    ax.axis("off")
                    st.pyplot(fig, width="stretch")
                    plt.close(fig)

# ── Features ───────────────────────────────────────────────────────────────
with tab_features:
    st.subheader("What does each layer see?")
    if st.session_state[skey("model_state")] is None:
        st.info("Train a model first (Training tab).")
    else:
        device = torch.device("cpu")
        model = cfg.model_cls().to(device)
        model.load_state_dict(st.session_state[skey("model_state")])
        model.eval()

        c1, c2, c3 = st.columns([1, 1, 2])
        split_name = c1.selectbox(
            "Split", ["Test", "Validation", "Train"], index=0, key=skey("feat_split")
        )
        cls_name = c2.selectbox(
            "Class", CLASS_NAMES, index=0, key=skey("feat_class")
        )
        ds = {"Train": train_ds, "Validation": val_ds, "Test": test_ds}[split_name]

        labels = np.array(ds.labels)
        cls_idx = CLASS_NAMES.index(cls_name)
        candidates = np.where(labels == cls_idx)[0]
        if len(candidates) == 0:
            st.warning("No samples of that class in this split.")
            st.stop()
        sample_pos = c3.slider(
            "Sample", 0, len(candidates) - 1, 0, key=skey("feat_sample")
        )
        idx = int(candidates[sample_pos])
        img, label = ds[idx]
        x = img.unsqueeze(0)

        with torch.no_grad():
            logits = model(x)
            probs = torch.softmax(logits, dim=1)[0].numpy()
            pred = int(logits.argmax(dim=1).item())

        left, right = st.columns([1, 2])
        with left:
            fig, ax = plt.subplots(figsize=(3, 3))
            imshow_sample(ax, img, cfg)
            ax.set_title(f"input — true: {CLASS_NAMES[int(label)]}")
            ax.axis("off")
            st.pyplot(fig, width="stretch")
            plt.close(fig)
            st.markdown(
                f"**Prediction:** `{CLASS_NAMES[pred]}` "
                f"({'✓' if pred == int(label) else '✗'})"
            )
            st.bar_chart(pd.DataFrame({"prob": probs}, index=CLASS_NAMES))
        with right:
            st.markdown("**First-layer learned filters**")
            fig = first_layer_filters_fig(model)
            st.pyplot(fig, width="stretch")
            plt.close(fig)

        st.markdown("---")
        st.markdown("**Activations after each layer**")
        show_all = st.checkbox(
            "Show every layer (conv / relu / pool)", value=False,
            key=skey("feat_show_all"),
            help="Off = only after each maxpool (3 stages). On = all 9 layers.",
        )
        acts = collect_activations(model, x)
        if not show_all:
            acts = [(n, h) for n, h in acts if "pool" in n]

        cmap_choice = st.selectbox(
            "Colormap", ["viridis", "magma", "gray", "inferno"],
            index=0, key=skey("feat_cmap"),
        )
        cols = st.columns(min(3, len(acts)))
        for i, (name, h) in enumerate(acts):
            with cols[i % len(cols)]:
                grid = tile_maps(h[0].numpy())
                fig, ax = plt.subplots(figsize=(3, 3))
                ax.imshow(grid, cmap=cmap_choice)
                ax.set_title(f"{name}\nshape {tuple(h.shape[1:])}", fontsize=9)
                ax.axis("off")
                st.pyplot(fig, width="stretch")
                plt.close(fig)

# ── ONNX ───────────────────────────────────────────────────────────────────
with tab_onnx:
    st.subheader("Export to ONNX & evaluate")
    st.markdown(
        "ONNX is the portable neural-net format you'd deploy to the robot: "
        "export the trained PyTorch weights, then run them with "
        "[ONNX Runtime](https://onnxruntime.ai/) (no PyTorch needed at inference)."
    )

    if st.session_state[skey("model_state")] is None:
        st.info("Train a model first (Training tab).")
    else:
        c1, c2 = st.columns([1, 2])
        with c1:
            if st.button("⬇ Export model → ONNX", type="primary", key=skey("export")):
                model = cfg.model_cls()
                model.load_state_dict(st.session_state[skey("model_state")])
                model.eval()
                dummy = torch.randn(*ONNX_INPUT_SHAPE)
                torch.onnx.export(
                    model, dummy, str(ONNX_PATH),
                    input_names=["input"], output_names=["logits"],
                    dynamic_axes={"input": {0: "batch"}, "logits": {0: "batch"}},
                    opset_version=18,
                )
                st.success(f"Exported `{ONNX_PATH.name}`")

        with c2:
            if ONNX_PATH.exists():
                size_kb = ONNX_PATH.stat().st_size / 1024
                st.success(
                    f"ONNX file on disk: `{ONNX_PATH}` — **{size_kb:.1f} KB**"
                )
                st.caption(
                    f"Input shape: `{ONNX_INPUT_SHAPE}` (NCHW, batch dim dynamic)"
                )
            else:
                st.info("No ONNX file yet — click export.")

        if ONNX_PATH.exists():
            st.markdown("---")
            st.subheader("Run the ONNX model on the test set")
            st.caption(
                "This uses ONNX Runtime (CPU) — completely independent of PyTorch."
            )

            if st.button("▶ Evaluate ONNX on test set", key=skey("eval_onnx")):
                import onnxruntime as ort

                sess = ort.InferenceSession(
                    str(ONNX_PATH), providers=["CPUExecutionProvider"]
                )
                input_name = sess.get_inputs()[0].name

                t0 = time.time()
                batch_np = np.stack(
                    [test_ds[i][0].numpy() for i in range(len(test_ds))]
                ).astype(np.float32)
                targets = np.array(
                    [int(test_ds[i][1]) for i in range(len(test_ds))]
                )
                logits = sess.run(None, {input_name: batch_np})[0]
                preds = logits.argmax(axis=1)
                dt = time.time() - t0

                acc = float((preds == targets).mean())
                c1, c2 = st.columns([1, 1])
                with c1:
                    st.metric("ONNX test accuracy", f"{acc * 100:.2f} %")
                    st.metric(
                        "Inference time", f"{dt * 1000:.0f} ms",
                        help=f"For {len(test_ds)} samples on CPU",
                    )
                    for cls_idx, cls_name in enumerate(CLASS_NAMES):
                        mask = targets == cls_idx
                        if mask.any():
                            cls_acc = (preds[mask] == cls_idx).mean()
                            st.write(f"• **{cls_name}**: {cls_acc * 100:.1f} %")
                with c2:
                    st.pyplot(
                        confusion_matrix_fig(preds, targets, cfg), width="stretch"
                    )

                torch_model = cfg.model_cls()
                torch_model.load_state_dict(st.session_state[skey("model_state")])
                torch_model.eval()
                with torch.no_grad():
                    torch_preds = (
                        torch_model(torch.from_numpy(batch_np))
                        .argmax(dim=1).numpy()
                    )
                agree = float((torch_preds == preds).mean())
                st.caption(
                    f"PyTorch vs ONNX agreement on test set: "
                    f"**{agree * 100:.2f} %** (should be ~100 %)."
                )

# ── ONNX Runtime (standalone) ──────────────────────────────────────────────
with tab_runtime:
    st.subheader("Run any ONNX model on a single image")
    st.markdown(
        "Upload an ONNX file (or reuse the one exported above) and feed it "
        "a single image — this path **never touches PyTorch** and mirrors "
        "what the robot would do at inference time."
    )

    src = st.radio(
        "ONNX model source",
        ["Use exported file", "Upload .onnx file"],
        horizontal=True, key=skey("rt_src"),
    )

    onnx_bytes = None
    if src == "Use exported file":
        if ONNX_PATH.exists():
            onnx_bytes = ONNX_PATH.read_bytes()
            st.caption(
                f"Using `{ONNX_PATH.name}` "
                f"({ONNX_PATH.stat().st_size / 1024:.1f} KB)"
            )
        else:
            st.info("No exported ONNX yet — train a model and export it first.")
    else:
        up = st.file_uploader("Upload .onnx", type=["onnx"], key=skey("rt_upl_m"))
        if up is not None:
            onnx_bytes = up.getvalue()
            st.caption(f"Loaded `{up.name}` ({len(onnx_bytes) / 1024:.1f} KB)")

    if onnx_bytes is not None:
        import onnxruntime as ort
        from PIL import Image

        sess = ort.InferenceSession(
            onnx_bytes, providers=["CPUExecutionProvider"]
        )
        input_name = sess.get_inputs()[0].name
        input_shape = sess.get_inputs()[0].shape
        st.caption(f"Model input: `{input_name}` shape `{input_shape}`")

        st.markdown("---")
        st.markdown("**Pick an image**")
        img_src = st.radio(
            "Image source",
            ["Test sample", "Upload image"],
            horizontal=True, key=skey("rt_img_src"),
        )

        x_np = None
        display_img = None
        true_label = None

        if img_src == "Test sample":
            sample_idx = st.slider(
                "Test index", 0, len(test_ds) - 1, 0, key=skey("rt_idx")
            )
            img, true_label = test_ds[int(sample_idx)]
            x_np = img.unsqueeze(0).numpy().astype(np.float32)
            display_img = to_display(img, cfg)
        else:
            up_img = st.file_uploader(
                f"Upload an image (will be resized to {cfg.upload_size[0]}×"
                f"{cfg.upload_size[1]} {cfg.upload_pil_mode})",
                type=["png", "jpg", "jpeg", "bmp"],
                key=skey("rt_img_file"),
            )
            if up_img is not None:
                pil = (
                    Image.open(up_img)
                    .convert(cfg.upload_pil_mode)
                    .resize(cfg.upload_size)
                )
                arr = np.asarray(pil, dtype=np.float32) / 255.0
                if cfg.is_rgb:
                    x_np = arr.transpose(2, 0, 1)[None, :, :, :]
                    display_img = arr
                else:
                    x_np = arr[None, None, :, :]
                    display_img = arr

        if x_np is not None:
            c1, c2 = st.columns([1, 2])
            with c1:
                fig, ax = plt.subplots(figsize=(3, 3))
                if cfg.is_rgb:
                    ax.imshow(display_img)
                else:
                    ax.imshow(display_img, cmap="gray", vmin=0, vmax=1)
                title = "input"
                if true_label is not None:
                    title += f"\ntrue: {CLASS_NAMES[int(true_label)]}"
                ax.set_title(title, fontsize=10)
                ax.axis("off")
                st.pyplot(fig, width="stretch")
                plt.close(fig)

            t0 = time.time()
            logits = sess.run(None, {input_name: x_np})[0][0]
            dt = (time.time() - t0) * 1000
            probs = np.exp(logits - logits.max())
            probs /= probs.sum()
            pred = int(probs.argmax())

            with c2:
                ok = true_label is None or pred == int(true_label)
                mark = "" if true_label is None else (" ✓" if ok else " ✗")
                st.markdown(f"### Prediction: `{CLASS_NAMES[pred]}`{mark}")
                st.caption(f"ONNX Runtime inference: {dt:.1f} ms (CPU)")
                st.bar_chart(pd.DataFrame({"prob": probs}, index=CLASS_NAMES))
