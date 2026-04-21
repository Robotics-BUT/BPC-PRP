"""Streamlit demo: peek inside a small Llama-style LLM.

Run with:
    streamlit run streamlit_app_llm.py

Loads a small pretrained transformer (default: SmolLM2-135M, a Llama-style
architecture with RMSNorm + RoPE + SwiGLU + multi-head attention) and lets
you explore the four things that make an LLM tick:

  🔤 Tokenize   - text → token IDs (the "alphabet" the model actually uses)
  🏗 Architecture - blocks, heads, params, KV cache shape
  🎯 Next-token - the model is just a next-token probability machine
  ✍️ Generate    - autoregressive sampling with temperature / top-k / top-p
  🔥 Attention   - per-layer, per-head attention heatmaps over your text

The first run downloads weights (~270 MB for the default) into the HF cache.
"""

from __future__ import annotations

import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import streamlit as st
import torch
from transformers import AutoModelForCausalLM, AutoTokenizer


HERE = Path(__file__).parent
ONNX_DIR = HERE / "onnx_exports"


def onnx_path_for(model_id: str) -> Path:
    """Where we save/load the ONNX file for this model id."""
    safe = model_id.replace("/", "__")
    return ONNX_DIR / f"{safe}.onnx"


MODELS = {
    "SmolLM2-135M  (Llama-style, ~270 MB)": "HuggingFaceTB/SmolLM2-135M",
    "SmolLM2-360M  (Llama-style, ~720 MB)": "HuggingFaceTB/SmolLM2-360M",
    "Qwen2.5-0.5B  (Llama-style, ~1.0 GB)": "Qwen/Qwen2.5-0.5B",
    "TinyLlama-1.1B (Llama-style, ~2.2 GB)": "TinyLlama/TinyLlama-1.1B-Chat-v1.0",
}


def pick_device() -> torch.device:
    if torch.backends.mps.is_available():
        return torch.device("mps")
    if torch.cuda.is_available():
        return torch.device("cuda")
    return torch.device("cpu")


@st.cache_resource(show_spinner="Loading model + tokenizer (downloads on first run)…")
def load_model(model_id: str):
    tok = AutoTokenizer.from_pretrained(model_id)
    if tok.pad_token_id is None:
        tok.pad_token_id = tok.eos_token_id
    model = AutoModelForCausalLM.from_pretrained(
        model_id,
        torch_dtype=torch.float32,
        attn_implementation="eager",  # required so output_attentions actually returns
    )
    model.eval()
    device = pick_device()
    model.to(device)
    return tok, model, device


# A small, varied palette for token highlighting.
PALETTE = [
    "#fde68a", "#bbf7d0", "#bae6fd", "#fbcfe8", "#ddd6fe",
    "#fecaca", "#fed7aa", "#a7f3d0", "#c7d2fe", "#fef08a",
]


def render_tokens(tok, ids: list[int]) -> str:
    """Build an HTML span sequence with one colored chip per token."""
    out = []
    for i, tid in enumerate(ids):
        piece = tok.decode([tid])
        # Replace newlines / tabs with visible glyphs to keep layout intact.
        display = (
            piece.replace("\n", "↵\n").replace("\t", "→")
        )
        bg = PALETTE[i % len(PALETTE)]
        # White-space-pre keeps spaces visible.
        out.append(
            f"<span style='background:{bg};padding:1px 3px;margin:1px;"
            f"border-radius:3px;white-space:pre;font-family:monospace;"
            f"font-size:13px'>{display}</span>"
        )
    return "".join(out)


# ────────────────────────────────────────────────────────────────────────────
# UI
# ────────────────────────────────────────────────────────────────────────────

st.set_page_config(page_title="LLM internals", layout="wide")
st.title("🦙 Inside a tiny Llama-style LLM")
st.caption(
    "Explore tokenization, the next-token distribution, sampling, and "
    "attention of a small pretrained transformer."
)

with st.sidebar:
    st.header("Model")
    model_label = st.selectbox("Model", list(MODELS.keys()), index=0)
    model_id = MODELS[model_label]

tok, model, device = load_model(model_id)
cfg = model.config

with st.sidebar:
    st.caption(f"Device: `{device}`")
    st.caption(f"Loaded: `{model_id}`")

(tab_tok, tab_arch, tab_next, tab_gen, tab_attn,
 tab_onnx, tab_runtime) = st.tabs(
    ["🔤 Tokenize", "🏗 Architecture", "🎯 Next-token", "✍️ Generate",
     "🔥 Attention", "📦 ONNX", "🔌 ONNX Runtime"]
)

# ── Tokenize ───────────────────────────────────────────────────────────────
with tab_tok:
    st.subheader("Text → tokens")
    st.markdown(
        "An LLM doesn't see characters — it sees **tokens** (~sub-words). "
        "Here's how the tokenizer chops up your text."
    )
    text = st.text_area(
        "Text", value="The quick brown fox jumps over the lazy dog.", height=120,
    )
    enc = tok(text, return_tensors="pt")
    ids = enc.input_ids[0].tolist()

    c1, c2, c3 = st.columns(3)
    c1.metric("Characters", len(text))
    c2.metric("Tokens", len(ids))
    c3.metric("Vocab size", f"{tok.vocab_size:,}")

    st.markdown("**Highlighted tokens**")
    st.markdown(render_tokens(tok, ids), unsafe_allow_html=True)

    st.markdown("**Token table**")
    df = pd.DataFrame({
        "pos": list(range(len(ids))),
        "id": ids,
        "string": [repr(tok.decode([i])) for i in ids],
    })
    st.dataframe(df, hide_index=True, use_container_width=True, height=300)

# ── Architecture ───────────────────────────────────────────────────────────
with tab_arch:
    st.subheader("What's actually inside")

    n_params = sum(p.numel() for p in model.parameters())
    facts = {
        "Architecture": cfg.architectures[0] if cfg.architectures else type(model).__name__,
        "Hidden size (d_model)": cfg.hidden_size,
        "Number of layers": cfg.num_hidden_layers,
        "Attention heads": cfg.num_attention_heads,
        "KV heads (GQA)": getattr(cfg, "num_key_value_heads", cfg.num_attention_heads),
        "Head dim": cfg.hidden_size // cfg.num_attention_heads,
        "MLP intermediate": getattr(cfg, "intermediate_size", "—"),
        "Vocab size": cfg.vocab_size,
        "Max context length": getattr(cfg, "max_position_embeddings", "—"),
        "Total parameters": f"{n_params:,}",
    }
    cols = st.columns(2)
    for i, (k, v) in enumerate(facts.items()):
        cols[i % 2].markdown(f"**{k}** &nbsp; `{v}`")

    st.markdown("---")
    st.markdown(
        "Modern Llama-style decoder block (repeated `num_hidden_layers` times):"
    )
    st.code(
        "x  → RMSNorm → MultiHeadAttention(RoPE, KV cache) → +x   (residual)\n"
        "   → RMSNorm → SwiGLU MLP                          → +x   (residual)\n",
        language="text",
    )
    with st.expander("Full PyTorch module tree"):
        st.code(str(model), language="text")

# ── Next-token ─────────────────────────────────────────────────────────────
with tab_next:
    st.subheader("The next-token distribution")
    st.markdown(
        "An LLM is, fundamentally, a function `prompt → probability over the next token`. "
        "Generation is just sampling from that distribution and feeding the choice back in."
    )
    prompt = st.text_input("Prompt", value="The capital of France is")
    top_n = st.slider("Show top-N candidates", 5, 30, 10)
    temp = st.slider("Temperature (only changes the *display* here)",
                     0.1, 2.0, 1.0, step=0.05)

    enc = tok(prompt, return_tensors="pt").to(device)
    with torch.no_grad():
        logits = model(**enc).logits[0, -1, :].float().cpu()
    probs = torch.softmax(logits / max(temp, 1e-6), dim=-1)
    top = torch.topk(probs, top_n)
    rows = pd.DataFrame({
        "token": [repr(tok.decode([int(i)])) for i in top.indices],
        "prob (%)": [float(p) * 100 for p in top.values],
    })
    c1, c2 = st.columns([1, 1])
    with c1:
        st.dataframe(rows, hide_index=True, use_container_width=True, height=400)
    with c2:
        fig, ax = plt.subplots(figsize=(5, max(3, top_n * 0.25)))
        labels = [r[1:-1] if len(r) > 2 else r for r in rows["token"]]
        ax.barh(range(top_n)[::-1], rows["prob (%)"], color="#3b82f6")
        ax.set_yticks(range(top_n)[::-1], labels, fontsize=8)
        ax.set_xlabel("probability (%)")
        ax.set_title("Top-N next tokens")
        fig.tight_layout()
        st.pyplot(fig, use_container_width=True)
        plt.close(fig)

# ── Generate ───────────────────────────────────────────────────────────────
with tab_gen:
    st.subheader("Autoregressive generation")
    prompt = st.text_area("Prompt", value="Once upon a time, in a small village,",
                          height=100)
    c1, c2, c3, c4 = st.columns(4)
    max_new = c1.slider("Max new tokens", 16, 400, 120, step=8)
    temperature = c2.slider("Temperature", 0.1, 1.5, 0.8, step=0.05)
    top_k = c3.slider("Top-k (0 = off)", 0, 100, 50)
    top_p = c4.slider("Top-p (1.0 = off)", 0.1, 1.0, 0.95, step=0.05)

    if st.button("✍️ Generate", type="primary"):
        enc = tok(prompt, return_tensors="pt").to(device)
        input_len = enc.input_ids.shape[1]
        torch.manual_seed(int(time.time()))
        out_box = st.empty()
        generated_ids = enc.input_ids.clone()
        past = None
        token_buf = []

        with torch.no_grad():
            cur = generated_ids
            for step in range(max_new):
                outputs = model(input_ids=cur, past_key_values=past, use_cache=True)
                past = outputs.past_key_values
                logits = outputs.logits[0, -1, :].float() / max(temperature, 1e-6)

                if top_k > 0 and top_k < logits.size(0):
                    v, _ = torch.topk(logits, top_k)
                    logits[logits < v[-1]] = -float("inf")

                if top_p < 1.0:
                    sorted_logits, sorted_idx = torch.sort(logits, descending=True)
                    sp = torch.softmax(sorted_logits, dim=-1)
                    cum = torch.cumsum(sp, dim=-1)
                    mask = cum > top_p
                    mask[1:] = mask[:-1].clone()
                    mask[0] = False
                    sorted_logits[mask] = -float("inf")
                    logits[sorted_idx] = sorted_logits

                probs = torch.softmax(logits, dim=-1)
                next_id = torch.multinomial(probs, 1)
                generated_ids = torch.cat(
                    [generated_ids, next_id.unsqueeze(0)], dim=1
                )
                cur = next_id.unsqueeze(0)
                token_buf.append(int(next_id))

                if next_id.item() == tok.eos_token_id:
                    break

                if step % 4 == 0 or step == max_new - 1:
                    text_so_far = tok.decode(generated_ids[0], skip_special_tokens=True)
                    out_box.text_area(
                        "Output (streaming)", value=text_so_far, height=400
                    )

        final_text = tok.decode(generated_ids[0], skip_special_tokens=True)
        out_box.text_area("Output", value=final_text, height=400)
        st.caption(
            f"Generated {len(token_buf)} new tokens "
            f"(input was {input_len} tokens)."
        )

# ── Attention ──────────────────────────────────────────────────────────────
with tab_attn:
    st.subheader("Attention heatmaps")
    st.markdown(
        "For each token (row), how much does the model **look at** every previous "
        "token (column)? Different heads in different layers learn different patterns: "
        "syntax, position, induction copies, etc."
    )

    probe = st.text_area(
        "Text to analyze", value="The cat sat on the mat because it was tired.",
        height=80,
    )
    enc = tok(probe, return_tensors="pt").to(device)
    ids = enc.input_ids[0].tolist()
    if len(ids) < 2:
        st.warning("Type at least two tokens.")
        st.stop()
    if len(ids) > 64:
        st.warning(f"Truncating to first 64 tokens (was {len(ids)}).")
        ids = ids[:64]
        enc = {k: v[:, :64] for k, v in enc.items()}

    with torch.no_grad():
        outputs = model(**enc, output_attentions=True)
    # tuple of length n_layers, each (1, n_heads, T, T)
    attns = outputs.attentions
    n_layers = len(attns)
    n_heads = attns[0].shape[1]
    tokens = [tok.decode([i]).replace("\n", "↵") for i in ids]
    short = [(t if len(t) <= 6 else t[:5] + "…") for t in tokens]

    c1, c2, c3 = st.columns([1, 1, 2])
    layer = c1.slider("Layer", 0, n_layers - 1, n_layers // 2)
    head_mode = c2.radio("View", ["Single head", "All heads (this layer)"], index=0)
    if head_mode == "Single head":
        head = c3.slider("Head", 0, n_heads - 1, 0)
        a = attns[layer][0, head].float().cpu().numpy()
        fig, ax = plt.subplots(figsize=(min(12, max(4, len(ids) * 0.45)),
                                         min(10, max(4, len(ids) * 0.45))))
        im = ax.imshow(a, cmap="magma", vmin=0, vmax=a.max())
        ax.set_xticks(range(len(ids)), short, rotation=60, fontsize=8, ha="right")
        ax.set_yticks(range(len(ids)), short, fontsize=8)
        ax.set_xlabel("attended token (key)")
        ax.set_ylabel("query token")
        ax.set_title(f"Layer {layer} · Head {head}")
        fig.colorbar(im, ax=ax, fraction=0.04, pad=0.02)
        fig.tight_layout()
        st.pyplot(fig, use_container_width=True)
        plt.close(fig)
    else:
        cols = 4
        rows = int(np.ceil(n_heads / cols))
        fig, axes = plt.subplots(rows, cols, figsize=(cols * 2.4, rows * 2.4))
        for h in range(n_heads):
            ax = np.array(axes).flat[h]
            a = attns[layer][0, h].float().cpu().numpy()
            ax.imshow(a, cmap="magma", vmin=0, vmax=a.max())
            ax.set_title(f"head {h}", fontsize=8)
            ax.set_xticks([])
            ax.set_yticks([])
        for h in range(n_heads, rows * cols):
            np.array(axes).flat[h].axis("off")
        fig.suptitle(f"Layer {layer} — all {n_heads} heads", y=1.0)
        fig.tight_layout()
        st.pyplot(fig, use_container_width=True)
        plt.close(fig)

    st.markdown("---")
    st.markdown("**Per-token attention weight summary** (averaged over all heads, this layer)")
    avg = attns[layer][0].mean(dim=0).float().cpu().numpy()
    df = pd.DataFrame(avg, index=tokens, columns=tokens)
    st.dataframe(
        df.style.background_gradient(cmap="magma", axis=None).format("{:.2f}"),
        use_container_width=True, height=320,
    )

# ── ONNX ───────────────────────────────────────────────────────────────────
with tab_onnx:
    st.subheader("Export to ONNX & run with ONNX Runtime")
    st.markdown(
        "Wrap the model as `input_ids → logits` (no KV cache) and export it "
        "to [ONNX](https://onnx.ai/). The exported file can be served by "
        "[ONNX Runtime](https://onnxruntime.ai/) with no PyTorch / HF dependency — "
        "useful for deployment on edge hardware."
    )
    st.warning(
        "ONNX export of a full LLM is slow (~30–60 s per 100 M params) and "
        "writes the weights as a second `.onnx.data` file (fp32 is ~4×param count)."
    )

    onnx_path = onnx_path_for(model_id)
    ONNX_DIR.mkdir(exist_ok=True)

    c1, c2 = st.columns([1, 2])
    with c1:
        export_clicked = st.button("⬇ Export current model → ONNX", type="primary")
    with c2:
        if onnx_path.exists():
            mb = onnx_path.stat().st_size / 1e6
            data_path = Path(str(onnx_path) + ".data")
            data_mb = data_path.stat().st_size / 1e6 if data_path.exists() else 0.0
            st.success(
                f"On disk: `{onnx_path.name}` ({mb:.1f} MB graph + "
                f"{data_mb:.1f} MB weights)"
            )
        else:
            st.info("No ONNX file yet for this model — click export.")

    if export_clicked:
        class _OnnxWrapper(torch.nn.Module):
            def __init__(self, m): super().__init__(); self.m = m
            def forward(self, input_ids):
                return self.m(input_ids=input_ids, use_cache=False).logits

        cpu_model = AutoModelForCausalLM.from_pretrained(
            model_id, torch_dtype=torch.float32, attn_implementation="eager",
        )
        cpu_model.eval()
        wrapper = _OnnxWrapper(cpu_model).eval()
        dummy = torch.randint(0, cfg.vocab_size, (1, 8), dtype=torch.long)

        prog = st.progress(0.0, text="Tracing + exporting…")
        t0 = time.time()
        torch.onnx.export(
            wrapper, (dummy,), str(onnx_path),
            input_names=["input_ids"], output_names=["logits"],
            dynamic_axes={
                "input_ids": {0: "batch", 1: "seq"},
                "logits": {0: "batch", 1: "seq"},
            },
            opset_version=18,
        )
        prog.progress(1.0, text=f"Done in {time.time() - t0:.1f} s")
        del cpu_model, wrapper
        st.success(f"Exported `{onnx_path}`")
        st.rerun()

    if onnx_path.exists():
        st.markdown("---")
        st.subheader("Next-token prediction via ONNX Runtime")
        st.caption(
            "Runs a single forward pass through ONNX Runtime and compares "
            "the top next-token candidates against PyTorch."
        )

        onnx_prompt = st.text_input(
            "Prompt", value="The capital of France is", key="onnx_prompt"
        )
        onnx_topn = st.slider("Top-N", 5, 20, 10, key="onnx_topn")

        if st.button("▶ Run ONNX next-token"):
            import onnxruntime as ort

            sess = ort.InferenceSession(
                str(onnx_path), providers=["CPUExecutionProvider"]
            )
            input_name = sess.get_inputs()[0].name

            ids = tok(onnx_prompt, return_tensors="pt").input_ids

            t0 = time.time()
            onnx_logits = sess.run(None, {input_name: ids.numpy()})[0]
            dt_onnx = time.time() - t0
            onnx_last = onnx_logits[0, -1]

            t0 = time.time()
            with torch.no_grad():
                pt_logits = model(ids.to(device)).logits[0, -1].float().cpu().numpy()
            dt_pt = time.time() - t0

            onnx_probs = torch.softmax(torch.from_numpy(onnx_last), dim=-1).numpy()
            pt_probs = torch.softmax(torch.from_numpy(pt_logits), dim=-1).numpy()

            c1, c2, c3 = st.columns(3)
            c1.metric("ONNX Runtime", f"{dt_onnx * 1000:.0f} ms")
            c2.metric("PyTorch", f"{dt_pt * 1000:.0f} ms")
            c3.metric(
                "Top-1 match",
                "✓" if onnx_last.argmax() == pt_logits.argmax() else "✗",
            )

            top_onnx = onnx_probs.argsort()[::-1][:onnx_topn]
            top_pt = pt_probs.argsort()[::-1][:onnx_topn]
            df = pd.DataFrame({
                "ONNX token": [repr(tok.decode([int(i)])) for i in top_onnx],
                "ONNX prob (%)": [float(onnx_probs[i]) * 100 for i in top_onnx],
                "PyTorch token": [repr(tok.decode([int(i)])) for i in top_pt],
                "PyTorch prob (%)": [float(pt_probs[i]) * 100 for i in top_pt],
            })
            st.dataframe(df, hide_index=True, use_container_width=True)

            agree = len(set(top_onnx.tolist()) & set(top_pt.tolist())) / onnx_topn
            st.caption(
                f"Top-{onnx_topn} overlap: **{agree * 100:.0f} %** — "
                f"max |Δlogit| = {float(np.abs(onnx_last - pt_logits).max()):.4f}"
            )

# ── ONNX Runtime (standalone) ──────────────────────────────────────────────
with tab_runtime:
    st.subheader("Run any ONNX model on your text")
    st.markdown(
        "Point at an ONNX file on disk (including ones exported by this or "
        "other tools) and run next-token inference through "
        "[ONNX Runtime](https://onnxruntime.ai/) — no PyTorch involved."
    )
    st.caption(
        "LLM ONNX exports store weights in a `.onnx.data` sidecar, so we load "
        "by **path** (not by upload) to keep graph and weights side-by-side."
    )

    default_path = str(onnx_path_for(model_id))
    existing = sorted(str(p) for p in ONNX_DIR.glob("*.onnx")) if ONNX_DIR.exists() else []
    path_str = st.text_input(
        "Path to .onnx file",
        value=default_path,
        help="Absolute or relative to this script's directory.",
    )
    if existing:
        st.caption("Found on disk: " + ", ".join(f"`{Path(p).name}`" for p in existing))

    onnx_file = Path(path_str).expanduser()
    if not onnx_file.is_absolute():
        onnx_file = (HERE / onnx_file).resolve()

    if not onnx_file.exists():
        st.info(f"File not found: `{onnx_file}` — export one from the ONNX tab first.")
    else:
        st.caption(f"Using `{onnx_file}` ({onnx_file.stat().st_size / 1e6:.1f} MB)")

        rt_prompt = st.text_area(
            "Prompt",
            value="The capital of France is",
            height=100,
            key="runtime_prompt",
        )
        c1, c2 = st.columns(2)
        rt_topn = c1.slider("Top-N", 5, 30, 10, key="runtime_topn")
        rt_temp = c2.slider("Temperature", 0.1, 2.0, 1.0, step=0.05, key="runtime_temp")

        if st.button("▶ Run ONNX inference", type="primary"):
            import onnxruntime as ort

            t0 = time.time()
            sess = ort.InferenceSession(
                str(onnx_file), providers=["CPUExecutionProvider"]
            )
            dt_load = (time.time() - t0) * 1000
            input_name = sess.get_inputs()[0].name

            ids = tok(rt_prompt, return_tensors="np").input_ids.astype(np.int64)

            t0 = time.time()
            logits = sess.run(None, {input_name: ids})[0]
            dt_run = (time.time() - t0) * 1000
            last = logits[0, -1].astype(np.float32)

            probs = torch.softmax(
                torch.from_numpy(last) / max(rt_temp, 1e-6), dim=-1
            ).numpy()
            top = probs.argsort()[::-1][:rt_topn]

            c1, c2, c3 = st.columns(3)
            c1.metric("Session load", f"{dt_load:.0f} ms")
            c2.metric("Forward pass", f"{dt_run:.0f} ms")
            c3.metric("Tokens in prompt", ids.shape[1])

            rows = pd.DataFrame({
                "token": [repr(tok.decode([int(i)])) for i in top],
                "id": [int(i) for i in top],
                "prob (%)": [float(probs[i]) * 100 for i in top],
            })
            st.dataframe(rows, hide_index=True, use_container_width=True)

            next_id = int(top[0])
            st.markdown(
                f"**Greedy next token:** {repr(tok.decode([next_id]))} "
                f"(id={next_id}, p={probs[next_id] * 100:.1f} %)"
            )
            st.caption(
                f"Prompt + greedy token → `"
                f"{rt_prompt + tok.decode([next_id])}`"
            )
