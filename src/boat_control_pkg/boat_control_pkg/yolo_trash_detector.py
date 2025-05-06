#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2


class YOLOTrashDetector(Node):
    """
    ROS 2 node that subscribes to a camera feed, runs YOLOv8 detection, and logs
    detections of interest (trash, bird, boat, human). Optionally publishes detection strings.
    """

    def __init__(self):
        super().__init__('yolo_trash_detector')

        self.bridge = CvBridge()

        # Load your custom trained YOLOv8 model
        model_path = model_path = '/home/anushka-satav/yolov8n.pt'

        self.model = YOLO(model_path)

        # Your actual camera topic here
        camera_topic = '/camera/image_raw'

        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )

        self.detection_pub = self.create_publisher(String, '/boat/yolo_detections', 10)

        self.relevant_classes = ['trash', 'bird', 'boat', 'human']
        self.get_logger().info(f"🚀 YOLOv8 Detector Node Initialized (listening on {camera_topic})")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        results = self.model.predict(source=frame, verbose=False)[0]

        for box in results.boxes:
            cls_id = int(box.cls[0])
            cls_name = self.model.names[cls_id]
            conf = float(box.conf[0])

            if cls_name in self.relevant_classes:
                message = f"📦 Detected: {cls_name} (confidence: {conf:.2f})"
                self.get_logger().info(message)
                self.detection_pub.publish(String(data=message))


def main(args=None):
    rclpy.init(args=args)
    node = YOLOTrashDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

