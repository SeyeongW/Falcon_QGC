#!/usr/bin/env python3
"""
Camera red marker perception node for robot-aircraft mission.

Run:
    python3 camera.py

This file directly subscribes to the Gazebo gimbal camera ROS2 image topic,
detects red survivor/target marker, and publishes navigation-ready target info.

Published topics:
    /mission/target_info
    /mission/target_info_json
    /mission/target_debug_image
"""

import json
import math
from dataclasses import dataclass, asdict
from typing import Tuple

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
from cv_bridge import CvBridge


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@dataclass
class TargetNavInfo:
    detected: bool = False

    image_width: int = 0
    image_height: int = 0

    bbox_x: float = 0.0
    bbox_y: float = 0.0
    bbox_w: float = 0.0
    bbox_h: float = 0.0

    center_x_px: float = 0.0
    center_y_px: float = 0.0

    image_center_x_px: float = 0.0
    image_center_y_px: float = 0.0

    error_x_px: float = 0.0
    error_y_px: float = 0.0

    error_x_norm: float = 0.0
    error_y_norm: float = 0.0

    bearing_x_rad: float = 0.0
    bearing_y_rad: float = 0.0

    area_px: float = 0.0
    area_ratio: float = 0.0

    range_proxy: float = 0.0
    small_target: bool = False

    contours: int = 0
    largest_area: float = 0.0
    mask_pixels: int = 0
    hsv_pixels: int = 0
    dominance_pixels: int = 0


class RedMarkerNavigationNode(Node):
    def __init__(self):
        super().__init__("red_marker_navigation_node")

        # ============================================================
        # Default settings
        # python3 camera.py 만 실행해도 바로 동작하도록 기본값을 여기서 고정
        # ============================================================

        default_image_topic = (
            "/world/default/model/standard_vtol_0/model/gimbal_model/"
            "link/camera_link/sensor/camera/image"
        )

        self.declare_parameter("image_topic", default_image_topic)
        self.declare_parameter("target_info_topic", "/mission/target_info")
        self.declare_parameter("target_info_json_topic", "/mission/target_info_json")
        self.declare_parameter("debug_image_topic", "/mission/target_debug_image")

        self.declare_parameter("show_window", True)
        self.declare_parameter("show_mask", True)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("log_interval_sec", 0.3)

        # Red HSV threshold.
        self.declare_parameter("lower1_h", 0)
        self.declare_parameter("upper1_h", 15)
        self.declare_parameter("lower2_h", 165)
        self.declare_parameter("upper2_h", 180)
        self.declare_parameter("min_s", 50)
        self.declare_parameter("min_v", 30)

        # BGR red dominance threshold.
        self.declare_parameter("red_min_value", 40)
        self.declare_parameter("red_dominance_ratio", 1.05)

        # Robust small target handling.
        self.declare_parameter("min_area", 1.0)
        self.declare_parameter("kernel_size", 3)
        self.declare_parameter("dilate_iterations", 4)
        self.declare_parameter("display_box_min_size", 50)

        # Approximate camera FOV.
        self.declare_parameter("camera_hfov_deg", 90.0)
        self.declare_parameter("camera_vfov_deg", 60.0)

        self.declare_parameter("range_proxy_gain", 1.0)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.target_info_topic = str(self.get_parameter("target_info_topic").value)
        self.target_info_json_topic = str(self.get_parameter("target_info_json_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)

        self.show_window = bool(self.get_parameter("show_window").value)
        self.show_mask = bool(self.get_parameter("show_mask").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.log_interval_sec = float(self.get_parameter("log_interval_sec").value)

        self.lower1_h = int(self.get_parameter("lower1_h").value)
        self.upper1_h = int(self.get_parameter("upper1_h").value)
        self.lower2_h = int(self.get_parameter("lower2_h").value)
        self.upper2_h = int(self.get_parameter("upper2_h").value)
        self.min_s = int(self.get_parameter("min_s").value)
        self.min_v = int(self.get_parameter("min_v").value)

        self.red_min_value = int(self.get_parameter("red_min_value").value)
        self.red_dominance_ratio = float(self.get_parameter("red_dominance_ratio").value)

        self.min_area = float(self.get_parameter("min_area").value)
        self.kernel_size = int(self.get_parameter("kernel_size").value)
        self.dilate_iterations = int(self.get_parameter("dilate_iterations").value)
        self.display_box_min_size = int(self.get_parameter("display_box_min_size").value)

        self.camera_hfov_rad = math.radians(float(self.get_parameter("camera_hfov_deg").value))
        self.camera_vfov_rad = math.radians(float(self.get_parameter("camera_vfov_deg").value))
        self.range_proxy_gain = float(self.get_parameter("range_proxy_gain").value)

        if self.kernel_size < 1:
            self.kernel_size = 1
        if self.kernel_size % 2 == 0:
            self.kernel_size += 1

        self.bridge = CvBridge()

        self.sub_image = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10,
        )

        self.pub_target_info = self.create_publisher(
            Float32MultiArray,
            self.target_info_topic,
            10,
        )

        self.pub_target_info_json = self.create_publisher(
            String,
            self.target_info_json_topic,
            10,
        )

        self.pub_debug_image = self.create_publisher(
            Image,
            self.debug_image_topic,
            10,
        )

        self.last_log_time = self.get_clock().now()

        self.get_logger().info("==================================================")
        self.get_logger().info("Camera Red Marker Perception Node Started")
        self.get_logger().info(f"image_topic             : {self.image_topic}")
        self.get_logger().info(f"target_info_topic       : {self.target_info_topic}")
        self.get_logger().info(f"target_info_json_topic  : {self.target_info_json_topic}")
        self.get_logger().info(f"debug_image_topic       : {self.debug_image_topic}")
        self.get_logger().info(f"show_window             : {self.show_window}")
        self.get_logger().info(f"show_mask               : {self.show_mask}")
        self.get_logger().info(f"min_area                : {self.min_area}")
        self.get_logger().info(f"kernel_size             : {self.kernel_size}")
        self.get_logger().info(f"dilate_iterations       : {self.dilate_iterations}")
        self.get_logger().info(f"display_box_min_size    : {self.display_box_min_size}")
        self.get_logger().info(f"red_min_value           : {self.red_min_value}")
        self.get_logger().info(f"red_dominance_ratio     : {self.red_dominance_ratio}")
        self.get_logger().info(f"camera_hfov_deg         : {math.degrees(self.camera_hfov_rad):.1f}")
        self.get_logger().info(f"camera_vfov_deg         : {math.degrees(self.camera_vfov_rad):.1f}")
        self.get_logger().info("==================================================")

    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"cv_bridge conversion failed: {exc}")
            return

        info, annotated, mask = self.detect_red_marker(frame)

        self.publish_target_info(info)

        if self.publish_debug_image:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                debug_msg.header = msg.header
                self.pub_debug_image.publish(debug_msg)
            except Exception as exc:
                self.get_logger().warn(f"failed to publish debug image: {exc}")

        self.log_info_throttled(info)

        if self.show_window:
            cv2.imshow("Survivor Navigation", annotated)
            if self.show_mask:
                cv2.imshow("Red Mask Debug", mask)
            cv2.waitKey(1)

    def detect_red_marker(self, frame: np.ndarray) -> Tuple[TargetNavInfo, np.ndarray, np.ndarray]:
        height, width = frame.shape[:2]
        image_cx = width / 2.0
        image_cy = height / 2.0

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red_1 = np.array([self.lower1_h, self.min_s, self.min_v], dtype=np.uint8)
        upper_red_1 = np.array([self.upper1_h, 255, 255], dtype=np.uint8)

        lower_red_2 = np.array([self.lower2_h, self.min_s, self.min_v], dtype=np.uint8)
        upper_red_2 = np.array([self.upper2_h, 255, 255], dtype=np.uint8)

        hsv_mask_1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        hsv_mask_2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
        hsv_mask = cv2.bitwise_or(hsv_mask_1, hsv_mask_2)

        b, g, r = cv2.split(frame)

        r_float = r.astype(np.float32)
        g_float = g.astype(np.float32)
        b_float = b.astype(np.float32)

        dominance_mask_bool = (
            (r_float > float(self.red_min_value))
            & (r_float > g_float * self.red_dominance_ratio)
            & (r_float > b_float * self.red_dominance_ratio)
        )
        dominance_mask = (dominance_mask_bool.astype(np.uint8)) * 255

        combined_mask = cv2.bitwise_or(hsv_mask, dominance_mask)

        kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)

        if self.dilate_iterations > 0:
            combined_mask = cv2.dilate(
                combined_mask,
                kernel,
                iterations=self.dilate_iterations,
            )

        contours, _ = cv2.findContours(
            combined_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        hsv_pixels = int(cv2.countNonZero(hsv_mask))
        dominance_pixels = int(cv2.countNonZero(dominance_mask))
        mask_pixels = int(cv2.countNonZero(combined_mask))

        annotated = frame.copy()
        self.draw_image_center(annotated, image_cx, image_cy)

        base_info = TargetNavInfo(
            detected=False,
            image_width=width,
            image_height=height,
            image_center_x_px=image_cx,
            image_center_y_px=image_cy,
            contours=len(contours),
            largest_area=0.0,
            mask_pixels=mask_pixels,
            hsv_pixels=hsv_pixels,
            dominance_pixels=dominance_pixels,
        )

        if mask_pixels == 0 and len(contours) == 0:
            cv2.putText(
                annotated,
                "SURVIVOR: NOT DETECTED",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 0, 255),
                2,
            )
            return base_info, annotated, combined_mask

        if not contours:
            moments = cv2.moments(combined_mask)
            if moments["m00"] <= 0.0:
                cv2.putText(
                    annotated,
                    "SURVIVOR: NOT DETECTED",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (0, 0, 255),
                    2,
                )
                return base_info, annotated, combined_mask

            cx = moments["m10"] / moments["m00"]
            cy = moments["m01"] / moments["m00"]
            bbox_x = cx
            bbox_y = cy
            bbox_w = 1.0
            bbox_h = 1.0
            area = float(mask_pixels)
        else:
            largest_contour = max(contours, key=cv2.contourArea)
            area = float(cv2.contourArea(largest_contour))

            x, y, w, h = cv2.boundingRect(largest_contour)

            bbox_x = float(x)
            bbox_y = float(y)
            bbox_w = float(w)
            bbox_h = float(h)

            cx = bbox_x + bbox_w / 2.0
            cy = bbox_y + bbox_h / 2.0

            if not np.isfinite(cx) or not np.isfinite(cy):
                moments = cv2.moments(combined_mask)
                if moments["m00"] > 0:
                    cx = moments["m10"] / moments["m00"]
                    cy = moments["m01"] / moments["m00"]

        detected = True
        small_target = bool(area < self.min_area)

        error_x_px = cx - image_cx
        error_y_px = cy - image_cy

        error_x_norm = error_x_px / (width / 2.0)
        error_y_norm = error_y_px / (height / 2.0)

        error_x_norm = clamp(error_x_norm, -1.0, 1.0)
        error_y_norm = clamp(error_y_norm, -1.0, 1.0)

        bearing_x_rad = math.atan(math.tan(self.camera_hfov_rad / 2.0) * error_x_norm)
        bearing_y_rad = math.atan(math.tan(self.camera_vfov_rad / 2.0) * error_y_norm)

        area_ratio = float(area) / float(width * height)
        if area_ratio > 1e-9:
            range_proxy = self.range_proxy_gain / math.sqrt(area_ratio)
        else:
            range_proxy = 0.0

        info = TargetNavInfo(
            detected=detected,
            image_width=width,
            image_height=height,
            bbox_x=bbox_x,
            bbox_y=bbox_y,
            bbox_w=bbox_w,
            bbox_h=bbox_h,
            center_x_px=cx,
            center_y_px=cy,
            image_center_x_px=image_cx,
            image_center_y_px=image_cy,
            error_x_px=error_x_px,
            error_y_px=error_y_px,
            error_x_norm=error_x_norm,
            error_y_norm=error_y_norm,
            bearing_x_rad=bearing_x_rad,
            bearing_y_rad=bearing_y_rad,
            area_px=area,
            area_ratio=area_ratio,
            range_proxy=range_proxy,
            small_target=small_target,
            contours=len(contours),
            largest_area=area,
            mask_pixels=mask_pixels,
            hsv_pixels=hsv_pixels,
            dominance_pixels=dominance_pixels,
        )

        self.draw_detection(annotated, info)

        return info, annotated, combined_mask

    def draw_image_center(self, image: np.ndarray, cx: float, cy: float) -> None:
        cx_i = int(cx)
        cy_i = int(cy)

        cv2.circle(image, (cx_i, cy_i), 5, (0, 255, 255), -1)
        cv2.line(image, (cx_i - 25, cy_i), (cx_i + 25, cy_i), (0, 255, 255), 1)
        cv2.line(image, (cx_i, cy_i - 25), (cx_i, cy_i + 25), (0, 255, 255), 1)

    def draw_detection(self, image: np.ndarray, info: TargetNavInfo) -> None:
        cx = int(info.center_x_px)
        cy = int(info.center_y_px)
        image_cx = int(info.image_center_x_px)
        image_cy = int(info.image_center_y_px)

        w = int(info.bbox_w)
        h = int(info.bbox_h)

        display_w = max(w, self.display_box_min_size)
        display_h = max(h, self.display_box_min_size)

        display_x = int(cx - display_w / 2)
        display_y = int(cy - display_h / 2)

        display_x = max(0, display_x)
        display_y = max(0, display_y)
        display_x2 = min(info.image_width - 1, display_x + display_w)
        display_y2 = min(info.image_height - 1, display_y + display_h)

        box_color = (0, 255, 0)
        if info.small_target:
            box_color = (0, 165, 255)

        cv2.rectangle(
            image,
            (display_x, display_y),
            (display_x2, display_y2),
            box_color,
            2,
        )

        cv2.circle(image, (cx, cy), 6, (0, 0, 255), -1)
        cv2.line(image, (image_cx, image_cy), (cx, cy), (0, 255, 255), 2)

        status = "SURVIVOR: DETECTED SMALL" if info.small_target else "SURVIVOR: DETECTED"

        cv2.putText(
            image,
            status,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            box_color,
            2,
        )

        cv2.putText(
            image,
            f"ex_px={info.error_x_px:.1f}, ey_px={info.error_y_px:.1f}",
            (20, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            image,
            f"ex_norm={info.error_x_norm:.3f}, ey_norm={info.error_y_norm:.3f}",
            (20, 105),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            image,
            f"bearing=({math.degrees(info.bearing_x_rad):.1f}, {math.degrees(info.bearing_y_rad):.1f}) deg",
            (20, 135),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            image,
            f"area={info.area_px:.1f}, ratio={info.area_ratio:.6f}, range_proxy={info.range_proxy:.2f}",
            (20, 165),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
        )

    def publish_target_info(self, info: TargetNavInfo) -> None:
        array_msg = Float32MultiArray()
        array_msg.data = [
            1.0 if info.detected else 0.0,
            float(info.center_x_px),
            float(info.center_y_px),
            float(info.error_x_px),
            float(info.error_y_px),
            float(info.error_x_norm),
            float(info.error_y_norm),
            float(info.bearing_x_rad),
            float(info.bearing_y_rad),
            float(info.bbox_x),
            float(info.bbox_y),
            float(info.bbox_w),
            float(info.bbox_h),
            float(info.area_px),
            float(info.area_ratio),
            float(info.range_proxy),
            1.0 if info.small_target else 0.0,
        ]
        self.pub_target_info.publish(array_msg)

        json_msg = String()
        json_msg.data = json.dumps(asdict(info), ensure_ascii=False)
        self.pub_target_info_json.publish(json_msg)

    def log_info_throttled(self, info: TargetNavInfo) -> None:
        now = self.get_clock().now()
        elapsed = (now - self.last_log_time).nanoseconds * 1e-9

        if elapsed < self.log_interval_sec:
            return

        if info.detected:
            self.get_logger().warn(
                "[TARGET_NAV] DETECTED | "
                f"center=({info.center_x_px:.1f},{info.center_y_px:.1f}) | "
                f"err_px=({info.error_x_px:.1f},{info.error_y_px:.1f}) | "
                f"err_norm=({info.error_x_norm:.3f},{info.error_y_norm:.3f}) | "
                f"bearing=({math.degrees(info.bearing_x_rad):.2f},{math.degrees(info.bearing_y_rad):.2f})deg | "
                f"bbox=({info.bbox_x:.0f},{info.bbox_y:.0f},{info.bbox_w:.0f},{info.bbox_h:.0f}) | "
                f"area={info.area_px:.1f} ratio={info.area_ratio:.6f} | "
                f"range_proxy={info.range_proxy:.2f} | "
                f"small={info.small_target} | "
                f"contours={info.contours} mask_pixels={info.mask_pixels}"
            )
        else:
            self.get_logger().info(
                "[TARGET_NAV] NOT DETECTED | "
                f"contours={info.contours} "
                f"largest_area={info.largest_area:.1f} "
                f"mask_pixels={info.mask_pixels} "
                f"hsv_pixels={info.hsv_pixels} "
                f"dominance_pixels={info.dominance_pixels}"
            )

        self.last_log_time = now

    def destroy_node(self) -> None:
        if self.show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = RedMarkerNavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
