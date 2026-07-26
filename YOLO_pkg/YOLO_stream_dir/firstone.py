#!/usr/bin/env python3

import cv2
from ultralytics import YOLO
# import time

# ==========================
# CONFIG
# ==========================


MODEL_PATH = "yolov8n.pt"
OUTPUT_FILE = "/home/dronepc/Videos/rec_yolo.mkv"

# ==========================
# GSTREAMER PIPELINES
# ==========================

# if STREAM_TYPE.lower() == "mjpeg":
#     gst_pipeline = (
#         'udpsrc port=5000 '
#         'caps="application/x-rtp,encoding-name=JPEG,payload=26" ! '
#         'rtpjpegdepay ! jpegdec ! videoconvert ! appsink drop=true'
#     )

# elif STREAM_TYPE.lower() == "h264":
#     gst_pipeline = (
#         'udpsrc port=5000 '
#         'caps="application/x-rtp,encoding-name=H264,payload=96" ! '
#         'rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! '
#         'appsink drop=true'
#     )

# else:
#     raise ValueError("STREAM_TYPE must be mjpeg or h264")

# # ==========================
# # OPEN STREAM
# # ==========================

# cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

cap = cv2.VideoCapture("/dev/video3")
if not cap.isOpened():
    raise RuntimeError("Failed to open stream")

# ==========================
# LOAD YOLO
# ==========================

model = YOLO(MODEL_PATH)

# ==========================
# GET FRAME SIZE
# ==========================

ret, frame = cap.read()

if not ret:
    raise RuntimeError("Failed to receive first frame")

height, width = frame.shape[:2]

print(f"Resolution: {width}x{height}")

# ==========================
# VIDEO WRITER
# ==========================

# bisa di ganti2
# fourcc = cv2.VideoWriter_fourcc(*"X264") # codec
fourcc = cv2.VideoWriter_fourcc(*"MJPG")

writer = cv2.VideoWriter(
    OUTPUT_FILE,
    fourcc, 
    30.0, #fps
    (width, height)
)

# ==========================
# MAIN LOOP
# ==========================

# fps_time = time.time()

while True:

    ret, frame = cap.read()

    if not ret:
        continue

    # IMPORTANT:
    # NO RESIZING
    results = model.predict(
        source=frame,
        verbose=False,
        classes = [0],
        device = "cuda"
    )

    annotated = results[0].plot()

    for box in results[0].boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])

        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2

        print(f"center=({cx}, {cy})")

    # fps = 1.0 / (time.time() - fps_time)
    # fps_time = time.time()

    # cv2.putText(
    #     annotated,
    #     f"FPS: {fps:.1f}",
    #     (20, 40),
    #     cv2.FONT_HERSHEY_SIMPLEX,
    #     1,
    #     (0, 255, 0),
    #     2,
    # )

    cv2.imshow("YOLO Stream", annotated)

    writer.write(annotated)

    key = cv2.waitKey(1)

    if key == ord("q"):
        break

# ==========================
# CLEANUP
# ==========================

writer.release()
cap.release()
cv2.destroyAllWindows()
