#!/usr/bin/env python3
"""Phase 15 (perception layer): subscribes to the simulated vehicle's real
camera feed (sensor_msgs/Image, published by gz-sim's own Sensors system
and bridged by a real, unmodified ros_gz_bridge -- see
host/gazebo/diff_drive_camera.sdf), runs a small pretrained object
detector on each frame, and publishes the result as a standard
vision_msgs/Detection2DArray -- the real ROS2 message type for this,
not a custom one, so any real ROS2 tool (rviz2, Foxglove, another node)
can consume it without knowing anything project-specific.

Model: YOLOv8n ("nano") via ultralytics, pretrained on COCO (80 classes,
including "person" -- what the walking actor in
host/gazebo/diff_drive_camera.sdf actually is, so this has something
real and detectable in view rather than an empty scene). Not trained or
fine-tuned here -- a real pretrained model run as-is, same "real,
unmodified" standard this project already holds the XRCE-DDS agent and
Gazebo bridge to.

Also publishes an annotated copy of the camera frame (boxes + labels
drawn directly on the image, /detections_image) alongside the raw
Detection2DArray -- rviz2 has no built-in Detection2DArray-over-image
overlay display, so the standard way to see "boxes overlaid on the feed"
in rviz2 itself is to view this annotated image topic directly with its
ordinary Image display, rather than requiring a project-specific rviz
plugin.

Runs in a dedicated Python venv (host/setup_perception_venv.sh) with
--system-site-packages, so it sees the system's rclpy/cv_bridge (from
`source /opt/ros/*/setup.bash`) while getting its own pinned
torch/torchvision/ultralytics/numpy on top -- see that script's header
comment for why a plain --user pip install doesn't work here
(PEP 668 "externally managed environment") and why numpy is pinned
below 2.0 in that venv specifically (a real torch/torchvision/matplotlib
ABI conflict found by testing, not guessed).

Usage:
    source /opt/ros/*/setup.bash
    source ~/perception_venv/bin/activate
    # camera bridge already running (host/run_perception.sh sets all of
    # this up together):
    python3 host/perception_node.py
"""
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

CONFIDENCE_THRESHOLD = 0.35


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")
        self.bridge = CvBridge()

        self.declare_parameter("model", "yolov8n.pt")
        model_name = self.get_parameter("model").get_parameter_value().string_value
        self.get_logger().info(f"loading {model_name} (pretrained COCO weights)...")
        # Imported lazily, after rclpy's own node is up, so a slow first-run
        # model download doesn't delay this node registering with the graph.
        from ultralytics import YOLO

        self.model = YOLO(model_name)
        self.get_logger().info(f"model loaded, {len(self.model.names)} classes")

        self.detections_pub = self.create_publisher(Detection2DArray, "/detections", 10)
        self.annotated_pub = self.create_publisher(Image, "/detections_image", 10)
        self.create_subscription(Image, "/camera", self._on_image, 10)

        self.frame_count = 0
        self.detection_count = 0

    def _on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:  # noqa: BLE001 -- log and drop this frame, keep the node alive
            self.get_logger().warn(f"image conversion failed: {e}")
            return

        results = self.model(frame, verbose=False, conf=CONFIDENCE_THRESHOLD)[0]

        detections = Detection2DArray()
        detections.header = msg.header

        annotated = frame.copy()
        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            class_id = int(box.cls[0].item())
            score = float(box.conf[0].item())
            class_name = self.model.names[class_id]

            det = Detection2D()
            det.header = msg.header
            det.bbox = BoundingBox2D()
            det.bbox.center.position.x = (x1 + x2) / 2.0
            det.bbox.center.position.y = (y1 + y2) / 2.0
            det.bbox.size_x = x2 - x1
            det.bbox.size_y = y2 - y1

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = class_name
            hyp.hypothesis.score = score
            det.results.append(hyp)
            detections.detections.append(det)

            p1 = (int(x1), int(y1))
            p2 = (int(x2), int(y2))
            cv2.rectangle(annotated, p1, p2, (0, 255, 0), 2)
            label = f"{class_name} {score:.2f}"
            cv2.putText(annotated, label, (p1[0], max(p1[1] - 6, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

        self.detections_pub.publish(detections)

        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)

        self.frame_count += 1
        self.detection_count += len(detections.detections)
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f"{self.frame_count} frames processed, "
                f"{len(detections.detections)} detection(s) this frame, "
                f"{self.detection_count} total"
            )


def main():
    rclpy.init()
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
