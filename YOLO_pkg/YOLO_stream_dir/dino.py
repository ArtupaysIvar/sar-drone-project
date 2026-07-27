#!/usr/bin/env python3

import sys
import cv2
import torch
import argparse
import numpy as np

# If the IDEA-Research/DINO repo isn't installed as a package, point at it directly:
sys.path.insert(0, "/home/dronepc/DINO")    
from models import build_model  # from repo

# ==========================
# CONFIG
# ==========================
MODEL_PATH = "/home/dronepc/sar_project_dir/YOLO_pkg/YOLO_stream_dir/ve_dino_export_20260720_155111/training/checkpoint.pth"
OUTPUT_FILE = "/home/dronepc/Videos/dino_raw_stream.avi"  # Avi pairs cleanly with MJPG
CONF_THRESH = 0.5

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# ==========================
# OPEN STREAM
# ==========================
cap = cv2.VideoCapture("/dev/video2")
if not cap.isOpened():
    raise RuntimeError("Failed to open video device")

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"Streaming Resolution: {width}x{height}")

fourcc = cv2.VideoWriter_fourcc(*"MJPG")
writer = cv2.VideoWriter(OUTPUT_FILE, fourcc, 30.0, (width, height))

# ==========================
# LOAD DINO-DETR MODEL
# ==========================
# allowlist argparse.Namespace since we trust this checkpoint (our own export)
torch.serialization.add_safe_globals([argparse.Namespace])
checkpoint = torch.load(MODEL_PATH, map_location=device, weights_only=False)

if "model" in checkpoint:
    state_dict = checkpoint["model"]
elif "state_dict" in checkpoint:
    state_dict = checkpoint["state_dict"]
else:
    state_dict = checkpoint

# The training args saved in the checkpoint are what define the architecture
# (backbone, num_queries, hidden_dim, etc.) — build_model needs them.
if "args" not in checkpoint:
    raise RuntimeError(
        "Checkpoint has no 'args' key — can't reconstruct model config. "
        "Load the matching config.json / args used at export time instead."
    )
train_args = checkpoint["args"]

model, criterion, postprocessors = build_model(train_args)
missing, unexpected = model.load_state_dict(state_dict, strict=False)
if missing:
    print(f"Warning: missing keys when loading state_dict: {missing}")
if unexpected:
    print(f"Warning: unexpected keys when loading state_dict: {unexpected}")

model = model.to(device)
model.eval()

# ==========================
# MAIN LOOP
# ==========================
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Stream stopped or frame lost. Exiting loop...")
            break

        # --- PREPROCESSING ---
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        input_size = (640, 640)
        img_resized = cv2.resize(img_rgb, input_size)
        img_normalized = img_resized.astype(np.float32) / 255.0
        tensor_img = torch.from_numpy(img_normalized).permute(2, 0, 1).unsqueeze(0).to(device)

        # --- INFERENCE ---
        with torch.no_grad():
            outputs = model(tensor_img)

        # --- POST-PROCESSING ---
        # DINO's postprocessors expect target sizes to rescale boxes back to original resolution
        orig_target_sizes = torch.tensor([[height, width]], device=device)
        results = postprocessors["bbox"](outputs, orig_target_sizes)[0]

        boxes = results["boxes"]
        scores = results["scores"]
        labels = results["labels"]

        for box, score, label in zip(boxes, scores, labels):
            if score < CONF_THRESH:
                continue
            x1, y1, x2, y2 = box.int().tolist()
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{label.item()}:{score:.2f}", (x1, max(y1 - 5, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            print(f"Object Center: ({cx}, {cy}) label={label.item()} conf={score:.2f}")

        cv2.imshow("Raw DINO Stream", frame)
        writer.write(frame)

        if cv2.waitKey(1) == ord("q"):
            break
finally:
    # Always release, even on exception, so /dev/video2 doesn't get left
    # with a queued buffer outstanding (the VIDIOC_QBUF error you saw).
    writer.release()
    cap.release()
    cv2.destroyAllWindows()