#!/usr/bin/env python3

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst

import cv2
import numpy as np

from datetime import datetime
from ultralytics import YOLO

# ============================================================
# CONFIG
# ============================================================

MODEL_PATH = "yolov8n.pt"

OUTPUT_FILE = (
    f"/home/dronepc/Videos/"
    f"rec_yolo1_{datetime.now().strftime('%Y%m%d_%H%M')}.mkv"
)

PIPELINE_STR = (
    "udpsrc port=5600 "
    "caps=application/x-rtp,media=video,encoding-name=H264,payload=96 ! "
    "rtph264depay ! "
    "h264parse ! "
    "avdec_h264 ! "
    "videoconvert ! "
    "video/x-raw,format=BGR ! "
    "appsink name=sink "
    "emit-signals=false "
    "sync=false "
    "max-buffers=1 "
    "drop=true"
)

# ============================================================
# GSTREAMER INITIALIZATION
# ============================================================

Gst.init(None)

pipeline = Gst.parse_launch(PIPELINE_STR)
sink = pipeline.get_by_name("sink")

pipeline.set_state(Gst.State.PLAYING)


# ============================================================
# FRAME READER
# ============================================================

def read_frame():
    sample = sink.emit("pull-sample")

    if sample is None:
        return None

    buf = sample.get_buffer()
    caps = sample.get_caps()

    width = caps.get_structure(0).get_value("width")
    height = caps.get_structure(0).get_value("height")

    success, mapinfo = buf.map(Gst.MapFlags.READ)

    if not success:
        return None

    frame = (
        np.frombuffer(mapinfo.data, dtype=np.uint8)
        .reshape((height, width, 3))
        .copy()
    )

    buf.unmap(mapinfo)

    return frame


# ============================================================
# LOAD YOLO
# ============================================================

print("Loading YOLO model...")
model = YOLO(MODEL_PATH)
print("YOLO model loaded")

# ============================================================
# WAIT FOR FIRST FRAME
# ============================================================

print("Waiting for video stream...")

frame = None

while frame is None:
    frame = read_frame()

height, width = frame.shape[:2]

print(f"Resolution: {width}x{height}")

# ============================================================
# VIDEO WRITER
# ============================================================

fourcc = cv2.VideoWriter_fourcc(*"MJPG")

writer = cv2.VideoWriter(
    OUTPUT_FILE,
    fourcc, 
    20.0,
    (width, height)
)

# ============================================================
# MAIN LOOP
# ============================================================

while True:

    frame = read_frame()

    if frame is None:
        continue

    results = model.predict(
        source=frame,
        verbose=False,
        classes=[0],      # person class only
        device="cuda",
    )

    annotated = results[0].plot()

    for box in results[0].boxes:

        x1, y1, x2, y2 = map(int, box.xyxy[0])

        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2

        # Use cx, cy here if needed
        # print(cx, cy)

    cv2.imshow("YOLO Stream", annotated)

    writer.write(annotated)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# ============================================================
# CLEANUP
# ============================================================

writer.release()

pipeline.set_state(Gst.State.NULL)

cv2.destroyAllWindows()