#!/usr/bin/env python3
"""Scene-description trial (Option 2): run small vision-language models on colour snapshots
from HearSight walk recordings (<walk>.hsr.frames/*.ppm) and write a side-by-side report.

Usage (from the repo root, with the VLM environment):
  /home/naveen/Hearsight/vlm_env/bin/python tools/vlm/describe_test.py \
      --frames /path/walk_xxx.hsr.frames --models smolvlm qwen2vl --every 4 --out vlm_runs

Models are read from /home/naveen/Hearsight/models_vlm/<name> (downloaded once).
Runs on the CPU (no GPU needed); times are PC times, not phone times.
"""
import argparse
import glob
import os
import time

import torch
from PIL import Image

MODEL_DIR = "/home/naveen/Hearsight/models_vlm"
MODELS = {
    "smolvlm": "SmolVLM-500M-Instruct",
    "qwen2vl": "Qwen2-VL-2B-Instruct",
}
# Small models echo long, multi-part prompts ("Obstacles with their side."): keep it short.
PROMPT = ("I am blind and walking forward. In one short sentence, what is in front of me "
          "and on my left and right?")


def load(name):
    from transformers import AutoModelForImageTextToText, AutoProcessor
    path = os.path.join(MODEL_DIR, MODELS[name])
    kw = {}
    if name == "smolvlm":
        # One 512 px view instead of many tiles: ~10x faster, enough for a room-scale scene.
        kw = {"do_image_splitting": False, "size": {"longest_edge": 512}}
    elif name == "qwen2vl":
        kw = {"min_pixels": 224 * 224, "max_pixels": 448 * 448}   # cap the image tokens
    proc = AutoProcessor.from_pretrained(path, **kw)
    model = AutoModelForImageTextToText.from_pretrained(path, torch_dtype=torch.float32)
    model.eval()
    return proc, model


def describe(proc, model, img, facts=None):
    text = PROMPT + (f"\nSensor facts (trust these): {facts}" if facts else "")
    msgs = [{"role": "user", "content": [{"type": "image"}, {"type": "text", "text": text}]}]
    prompt = proc.apply_chat_template(msgs, add_generation_prompt=True)
    inputs = proc(text=[prompt], images=[img], return_tensors="pt")
    t0 = time.time()
    with torch.no_grad():
        out = model.generate(**inputs, max_new_tokens=40, do_sample=False)
    dt = time.time() - t0
    answer = proc.batch_decode(out[:, inputs["input_ids"].shape[1]:], skip_special_tokens=True)[0].strip()
    return answer, dt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", nargs="+", required=True, help="one or more <walk>.hsr.frames folders or images")
    ap.add_argument("--models", nargs="+", default=["smolvlm", "qwen2vl"], choices=list(MODELS))
    ap.add_argument("--every", type=int, default=4, help="use every Nth snapshot (2 Hz -> every 4 = every 2 s)")
    ap.add_argument("--max", type=int, default=40, help="max images per run")
    ap.add_argument("--out", default="vlm_runs")
    a = ap.parse_args()
    imgs = []
    for f in a.frames:
        imgs += sorted(glob.glob(os.path.join(f, "*.ppm"))) if os.path.isdir(f) else [f]
    imgs = imgs[::a.every][:a.max]
    if not imgs:
        ap.error("no images found")
    os.makedirs(a.out, exist_ok=True)
    torch.set_num_threads(os.cpu_count() or 4)
    results = {}
    for name in a.models:
        print(f"== loading {name}", flush=True)
        proc, model = load(name)
        results[name] = []
        for p in imgs:
            ans, dt = describe(proc, model, Image.open(p).convert("RGB"))
            results[name].append((ans, dt))
            print(f"{name:8} {os.path.basename(p)} {dt:5.1f}s  {ans}", flush=True)
        del model
    # Report: one row per image, each model's answer and time; copy of the image for viewing.
    lines = ["# Scene-description trial (PC, CPU)", "",
             "Rate each answer: ✅ correct · 🟡 partly · ❌ wrong / invented.", "",
             "| Image | " + " | ".join(a.models) + " |", "|---|" + "---|" * len(a.models)]
    for i, p in enumerate(imgs):
        png = os.path.join(a.out, os.path.basename(p).replace(".ppm", ".png"))
        Image.open(p).convert("RGB").save(png)
        cells = [f"{results[m][i][0]} *({results[m][i][1]:.1f} s)*" for m in a.models]
        lines.append(f"| ![]({os.path.basename(png)}) | " + " | ".join(cells) + " |")
    for m in a.models:
        ts = [dt for _, dt in results[m]]
        lines.append(f"\n**{m}**: {len(ts)} images, average {sum(ts) / len(ts):.1f} s (PC CPU), "
                     f"max {max(ts):.1f} s, average {sum(len(x.split()) for x, _ in results[m]) / len(ts):.0f} words")
    with open(os.path.join(a.out, "report.md"), "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print("report:", os.path.join(a.out, "report.md"))


if __name__ == "__main__":
    main()
