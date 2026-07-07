
#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading

import cv2
# import numpy as np
from ultralytics import YOLO

from pixel_msgs.msg import PixelCoordinates
from ament_index_python.packages import get_package_share_directory


class GrabVideoSims(Node):

    def __init__(self):
        super().__init__('grab_video')

        self.model = YOLO('best.pt')
        # self.model.to("cpu")
        self.bridge = CvBridge()

        self.det1 = None
        self.det2 = None
        self.det3 = None

        self.latest_frames = {
            1: None,
            2: None,
            3: None
        }
        self.lock = threading.Lock()

        # subscription
        self.subscription1 = self.create_subscription(
            Image, '/world/custom/model/x500_mono_cam_down_1/link/camera_link/sensor/imager/image',
            lambda msg: self.camera_callback(msg, 1), 10)
        
        self.subscription2 = self.create_subscription(
            Image, '/world/custom/model/x500_mono_cam_down_2/link/camera_link/sensor/imager/image', 
            lambda msg: self.camera_callback(msg, 2), 10)

        self.subscription3 = self.create_subscription(
            Image, '/world/custom/model/x500_mono_cam_down_3/link/camera_link/sensor/imager/image', 
            lambda msg: self.camera_callback(msg, 3), 10)
        
        
        # publishers
        self.pub = {
            1: self.create_publisher(PixelCoordinates, 'pixel_topic1', 10),
            2: self.create_publisher(PixelCoordinates, 'pixel_topic2', 10),
            3: self.create_publisher(PixelCoordinates, 'pixel_topic3', 10),
        }
        # timer
        self.timer = self.create_timer(0.4, self.timer_callback)
        cv2.namedWindow("Drone 1", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Drone 2", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Drone 3", cv2.WINDOW_NORMAL)

    def camera_callback(self, msg, drone_id):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        with self.lock:
            self.latest_frames[drone_id] = frame


    def timer_callback(self):

        # Copy frames safely
        with self.lock:
            frames_copy = {}
            for k, v in self.latest_frames.items():
                if v is not None:
                    frames_copy[k] = v.copy()
                else:
                    frames_copy[k] = None

        # Process each drone
        for drone_id, frame in frames_copy.items():

            if frame is None:
                continue
            
            start_time = time.perf_counter()
            
            results = self.model.predict(frame, classes=[0, 2], verbose=False)
            
            end_time = time.perf_counter()


            inference_time_ms = (end_time - start_time) * 1000.0

            self.get_logger().info(
                f"Drone {drone_id}: YOLO inference = {inference_time_ms:.2f} ms")

            # Draw detections
            annotated_frame = results[0].plot()

            # Display window
            cv2.imshow(f"Drone {drone_id}", annotated_frame)

            # Publish detection if exists
            if len(results[0].boxes) > 0:
                x_mid, y_mid, _, _ = results[0].boxes.xywh[0]
                confidence = float(results[0].boxes.conf[0])

                msg = PixelCoordinates()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.u = float(x_mid)
                msg.v = float(y_mid)
                msg.confidence = confidence

                self.pub[drone_id].publish(msg)
            else:
                msg = PixelCoordinates()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.u = 0.0
                msg.v = 0.0
                msg.confidence = 0.0

                self.pub[drone_id].publish(msg)

        # Required for OpenCV window refresh
        cv2.waitKey(1)

        
def main(args=None):
    rclpy.init(args=args)
    node = GrabVideoSims()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
