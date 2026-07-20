#!/usr/bin/env python3
"""Publish a synthetic sensor_msgs/Image stream for testing the VTOL-GCS ROS bridge.

Publishes an animated 640x480 rgb8 image at 10 Hz on /fake/image (override with
argv[1]). Handy stand-in for the recognition camera while wiring the video panel.

    source /opt/ros/humble/setup.bash
    python3 custom/tools/fake_image.py [topic] [hz] [seconds]

`seconds` (default 0) auto-stops the publisher after that many seconds; 0 runs
until Ctrl+C. A finite duration is handy for scripted/self-terminating tests.
"""
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class FakeImagePublisher(Node):
    def __init__(self, topic: str, hz: float, seconds: float = 0.0) -> None:
        super().__init__("fake_image_publisher")
        self._pub = self.create_publisher(Image, topic, 10)
        if seconds > 0:
            self.create_timer(seconds, self._stop)
        self._w, self._h = 640, 480
        self._frame = 0
        # Precompute per-pixel x/y ramps once; each tick just adds the frame
        # offset (vectorized) so publishing stays cheap even at high rates.
        xr = np.arange(self._w, dtype=np.uint16)
        yr = np.arange(self._h, dtype=np.uint16)
        self._x = np.broadcast_to(xr, (self._h, self._w))
        self._y = np.broadcast_to(yr[:, None], (self._h, self._w))
        self._timer = self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(f"publishing {self._w}x{self._h} rgb8 @ {hz} Hz on {topic}")

    def _stop(self) -> None:
        raise KeyboardInterrupt

    def _tick(self) -> None:
        f = self._frame
        # Moving diagonal color gradient so the feed is visibly animated.
        img = np.empty((self._h, self._w, 3), dtype=np.uint8)
        img[..., 0] = (self._x + f) & 0xFF
        img[..., 1] = (self._y + f) & 0xFF
        img[..., 2] = (self._x + self._y + f) & 0xFF
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.height = self._h
        msg.width = self._w
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = self._w * 3
        msg.data = img.tobytes()
        self._pub.publish(msg)
        self._frame = (f + 4) & 0xFF


def main() -> None:
    topic = sys.argv[1] if len(sys.argv) > 1 else "/fake/image"
    hz = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    rclpy.init()
    node = FakeImagePublisher(topic, hz, seconds)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
