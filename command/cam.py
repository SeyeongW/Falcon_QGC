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

The node estimates the undirected body-axis orientation of the red survivor.
Heading is computed from PCA over red survivor pixels, not from the outer white
landing panel or an axis-aligned bounding box. The relative heading error is
appended to the published target info and, when valid, encoded as a yaw-only
quaternion in MAVLink LANDING_TARGET.q.
"""

import json
import math
import os
from dataclasses import dataclass, asdict
from typing import Tuple

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
from cv_bridge import CvBridge

os.environ.setdefault("MAVLINK20", "1")

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_axis_angle_rad(angle_rad: float) -> float:
    """Wrap an undirected line-axis angle to [-pi/2, pi/2]."""
    while angle_rad > math.pi / 2.0:
        angle_rad -= math.pi
    while angle_rad < -math.pi / 2.0:
        angle_rad += math.pi
    return angle_rad


def yaw_to_quaternion(yaw_rad: float) -> list:
    """Return a MAVLink w, x, y, z yaw-only quaternion."""
    half_yaw = 0.5 * yaw_rad
    return [math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)]


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

    orientation_valid: bool = False
    long_axis_angle_rad: float = 0.0
    long_axis_angle_deg: float = 0.0
    heading_error_rad: float = 0.0
    heading_error_deg: float = 0.0
    long_side_px: float = 0.0
    short_side_px: float = 0.0
    aspect_ratio: float = 0.0
    orientation_pixels: int = 0
    pca_eigen_ratio: float = 0.0

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
        self.declare_parameter("mavlink_target_url", "udpout:127.0.0.1:14580")

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

        # Survivor body-axis heading estimation using PCA over red pixels.
        # heading_reference_deg is the vehicle-forward direction in the image:
        # 0 deg means image-up, +90 deg means image-right.
        self.declare_parameter("heading_min_area_px", 50.0)
        self.declare_parameter("heading_min_aspect_ratio", 1.15)
        self.declare_parameter("heading_min_long_side_px", 12.0)
        self.declare_parameter("heading_min_pixels", 30)
        self.declare_parameter("heading_roi_margin_px", 8)
        self.declare_parameter("heading_reference_deg", 0.0)
        self.declare_parameter("heading_sign", 1.0)
        self.declare_parameter("send_heading_quaternion", True)
        self.declare_parameter("draw_axis_aligned_box", False)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.target_info_topic = str(self.get_parameter("target_info_topic").value)
        self.target_info_json_topic = str(self.get_parameter("target_info_json_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.mavlink_target_url = str(self.get_parameter("mavlink_target_url").value)

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

        self.heading_min_area_px = max(
            0.0, float(self.get_parameter("heading_min_area_px").value)
        )
        self.heading_min_aspect_ratio = max(
            1.0, float(self.get_parameter("heading_min_aspect_ratio").value)
        )
        self.heading_min_long_side_px = max(
            1.0, float(self.get_parameter("heading_min_long_side_px").value)
        )
        self.heading_min_pixels = max(
            3, int(self.get_parameter("heading_min_pixels").value)
        )
        self.heading_roi_margin_px = max(
            0, int(self.get_parameter("heading_roi_margin_px").value)
        )
        self.heading_reference_rad = math.radians(
            float(self.get_parameter("heading_reference_deg").value)
        )
        heading_sign_value = float(self.get_parameter("heading_sign").value)
        self.heading_sign = -1.0 if heading_sign_value < 0.0 else 1.0
        self.send_heading_quaternion = bool(
            self.get_parameter("send_heading_quaternion").value
        )
        self.draw_axis_aligned_box = bool(
            self.get_parameter("draw_axis_aligned_box").value
        )

        if self.kernel_size < 1:
            self.kernel_size = 1
        if self.kernel_size % 2 == 0:
            self.kernel_size += 1

        self.bridge = CvBridge()
        self.mavlink_master = None
        self.mavlink_enabled = False
        self.init_mavlink_sender()

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
        self.last_mavlink_debug_log_time = None

        self.get_logger().info("==================================================")
        self.get_logger().info("Camera Red Marker Perception Node Started")
        self.get_logger().info(f"image_topic             : {self.image_topic}")
        self.get_logger().info(f"target_info_topic       : {self.target_info_topic}")
        self.get_logger().info(f"target_info_json_topic  : {self.target_info_json_topic}")
        self.get_logger().info(f"debug_image_topic       : {self.debug_image_topic}")
        self.get_logger().info(f"mavlink_target_url      : {self.mavlink_target_url}")
        self.get_logger().info(f"mavlink_landing_target  : {self.mavlink_enabled}")
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
        self.get_logger().info(f"heading_min_area_px     : {self.heading_min_area_px:.1f}")
        self.get_logger().info(f"heading_min_aspect_ratio: {self.heading_min_aspect_ratio:.2f}")
        self.get_logger().info(f"heading_min_long_side_px: {self.heading_min_long_side_px:.1f}")
        self.get_logger().info(f"heading_min_pixels      : {self.heading_min_pixels}")
        self.get_logger().info(f"heading_roi_margin_px   : {self.heading_roi_margin_px}")
        self.get_logger().info(f"heading_reference_deg   : {math.degrees(self.heading_reference_rad):.1f}")
        self.get_logger().info(f"heading_sign            : {self.heading_sign:+.0f}")
        self.get_logger().info(f"send_heading_quaternion : {self.send_heading_quaternion}")
        self.get_logger().info(f"draw_axis_aligned_box   : {self.draw_axis_aligned_box}")
        self.get_logger().info("==================================================")

    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"cv_bridge conversion failed: {exc}")
            return

        info, annotated, mask = self.detect_red_marker(frame)

        self.publish_target_info(info)
        self.send_landing_target(info)

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

    def init_mavlink_sender(self) -> None:
        if mavutil is None:
            self.get_logger().error("pymavlink import failed; MAVLink LANDING_TARGET sender disabled")
            return

        try:
            self.mavlink_master = mavutil.mavlink_connection(self.mavlink_target_url)
            self.mavlink_enabled = True
            self.get_logger().info(
                "MAVLink LANDING_TARGET sender enabled. PX4 check: listener irlock_report 5"
            )
        except Exception as exc:
            self.mavlink_master = None
            self.mavlink_enabled = False
            self.get_logger().error(
                f"failed to open MAVLink target {self.mavlink_target_url}: {exc}; sender disabled"
            )

    def send_landing_target(self, info: TargetNavInfo) -> None:
        if not self.mavlink_enabled or self.mavlink_master is None:
            return

        if not info.detected:
            return

        try:
            now_msg = self.get_clock().now()
            time_usec = now_msg.nanoseconds // 1000
            frame = mavutil.mavlink.MAV_FRAME_BODY_NED
            angle_x = float(info.bearing_x_rad)
            angle_y = float(info.bearing_y_rad)
            target_type = 2
            position_valid = 0

            if (
                self.send_heading_quaternion
                and info.orientation_valid
                and math.isfinite(info.heading_error_rad)
            ):
                target_quaternion = yaw_to_quaternion(info.heading_error_rad)
            else:
                # PX4 treats a finite identity quaternion as a valid zero-degree
                # heading. Send NaNs so the receiver can mark heading invalid.
                target_quaternion = [math.nan, math.nan, math.nan, math.nan]

            self.mavlink_master.mav.landing_target_send(
                int(time_usec),
                0,
                frame,
                angle_x,
                angle_y,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                target_quaternion,
                target_type,
                position_valid,
            )
            self.log_landing_target_debug(
                time_usec,
                frame,
                angle_x,
                angle_y,
                target_quaternion,
                target_type,
                position_valid,
                info.orientation_valid,
                info.heading_error_rad,
            )
        except Exception as exc:
            self.get_logger().warn(f"failed to send MAVLink LANDING_TARGET: {exc}")

    def log_landing_target_debug(
        self,
        time_usec: int,
        frame: int,
        angle_x: float,
        angle_y: float,
        target_quaternion: list,
        target_type: int,
        position_valid: int,
        orientation_valid: bool,
        heading_error_rad: float,
    ) -> None:
        now = self.get_clock().now()

        if self.last_mavlink_debug_log_time is not None:
            elapsed = (now - self.last_mavlink_debug_log_time).nanoseconds * 1e-9
            if elapsed < 1.0:
                return

        msg = mavutil.mavlink.MAVLink_landing_target_message(
            int(time_usec),
            0,
            frame,
            angle_x,
            angle_y,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            target_quaternion,
            target_type,
            position_valid,
        )

        heading_text = (
            f"{math.degrees(heading_error_rad):.2f}deg"
            if orientation_valid and math.isfinite(heading_error_rad)
            else "INVALID"
        )

        self.get_logger().info(
            "[MAVLINK_TX] LANDING_TARGET | "
            f"msg_id={msg.get_msgId()} | "
            f"angle_x={angle_x:.6f} | "
            f"angle_y={angle_y:.6f} | "
            f"heading_error={heading_text} | "
            f"q=({target_quaternion[0]:.4f},{target_quaternion[1]:.4f},"
            f"{target_quaternion[2]:.4f},{target_quaternion[3]:.4f}) | "
            f"frame={frame} | "
            f"position_valid={position_valid} | "
            f"type={target_type} | "
            f"url={self.mavlink_target_url}"
        )
        self.last_mavlink_debug_log_time = now

    def estimate_survivor_body_axis(
        self,
        orientation_mask: np.ndarray,
        roi: Tuple[int, int, int, int],
    ) -> dict:
        """
        Estimate the survivor's undirected body axis using PCA over red pixels.

        The source is only the red-pixel mask inside the detected survivor ROI.
        The outer white panel and axis-aligned bounding box are never used for
        heading. The resulting angle is measured from image-up and is folded to
        [-pi/2, pi/2], because a body axis has 180-degree symmetry.
        """
        x0, y0, x1, y1 = roi

        result = {
            "valid": False,
            "center": None,
            "box": None,
            "axis_segment": None,
            "long_axis_angle_rad": 0.0,
            "heading_error_rad": 0.0,
            "long_side_px": 0.0,
            "short_side_px": 0.0,
            "aspect_ratio": 0.0,
            "orientation_pixels": 0,
            "pca_eigen_ratio": 0.0,
        }

        if x1 <= x0 or y1 <= y0:
            return result

        roi_mask = orientation_mask[y0:y1, x0:x1]
        ys, xs = np.nonzero(roi_mask)

        orientation_pixels = int(xs.size)
        result["orientation_pixels"] = orientation_pixels

        if orientation_pixels < 3:
            return result

        points = np.column_stack(
            (xs.astype(np.float64) + float(x0), ys.astype(np.float64) + float(y0))
        )

        center = np.mean(points, axis=0)
        centered = points - center

        covariance = (centered.T @ centered) / max(1, orientation_pixels - 1)

        if not np.all(np.isfinite(covariance)):
            return result

        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        order = np.argsort(eigenvalues)[::-1]
        eigenvalues = eigenvalues[order]
        eigenvectors = eigenvectors[:, order]

        major_value = float(max(eigenvalues[0], 0.0))
        minor_value = float(max(eigenvalues[1], 0.0))

        if major_value <= 1e-9:
            return result

        axis_vector = eigenvectors[:, 0].astype(np.float64)
        axis_norm = float(np.linalg.norm(axis_vector))

        if axis_norm <= 1e-9:
            return result

        axis_vector /= axis_norm

        # PCA returns an undirected eigenvector. Choose the image-up direction
        # only for stable visualization and sign convention.
        if axis_vector[1] > 0.0 or (
            abs(float(axis_vector[1])) < 1e-9 and axis_vector[0] < 0.0
        ):
            axis_vector = -axis_vector

        secondary_vector = np.array(
            [-axis_vector[1], axis_vector[0]], dtype=np.float64
        )

        major_projection = centered @ axis_vector
        minor_projection = centered @ secondary_vector

        major_min = float(np.min(major_projection))
        major_max = float(np.max(major_projection))
        minor_min = float(np.min(minor_projection))
        minor_max = float(np.max(minor_projection))

        long_side_px = major_max - major_min
        short_side_px = minor_max - minor_min

        if long_side_px <= 1e-6 or short_side_px <= 1e-6:
            return result

        aspect_ratio = long_side_px / short_side_px
        pca_eigen_ratio = math.sqrt(
            major_value / max(minor_value, 1e-9)
        )

        # Angle from image-up, positive toward image-right.
        long_axis_angle_rad = wrap_axis_angle_rad(
            math.atan2(float(axis_vector[0]), -float(axis_vector[1]))
        )
        heading_error_rad = wrap_axis_angle_rad(
            self.heading_sign
            * wrap_axis_angle_rad(
                long_axis_angle_rad - self.heading_reference_rad
            )
        )

        # PCA-aligned visual box around the actual red survivor pixels.
        corners = np.array(
            [
                center + axis_vector * major_min + secondary_vector * minor_min,
                center + axis_vector * major_max + secondary_vector * minor_min,
                center + axis_vector * major_max + secondary_vector * minor_max,
                center + axis_vector * major_min + secondary_vector * minor_max,
            ],
            dtype=np.float32,
        )

        axis_start = center + axis_vector * major_min
        axis_end = center + axis_vector * major_max

        valid = bool(
            orientation_pixels >= self.heading_min_pixels
            and long_side_px >= self.heading_min_long_side_px
            and aspect_ratio >= self.heading_min_aspect_ratio
            and math.isfinite(long_axis_angle_rad)
            and math.isfinite(heading_error_rad)
            and math.isfinite(pca_eigen_ratio)
        )

        result.update(
            {
                "valid": valid,
                "center": center.astype(np.float32),
                "box": corners,
                "axis_segment": (
                    (
                        int(round(float(axis_start[0]))),
                        int(round(float(axis_start[1]))),
                    ),
                    (
                        int(round(float(axis_end[0]))),
                        int(round(float(axis_end[1]))),
                    ),
                ),
                "long_axis_angle_rad": long_axis_angle_rad,
                "heading_error_rad": heading_error_rad,
                "long_side_px": long_side_px,
                "short_side_px": short_side_px,
                "aspect_ratio": aspect_ratio,
                "orientation_pixels": orientation_pixels,
                "pca_eigen_ratio": pca_eigen_ratio,
            }
        )

        return result

    def detect_red_marker(
        self, frame: np.ndarray
    ) -> Tuple[TargetNavInfo, np.ndarray, np.ndarray]:
        height, width = frame.shape[:2]
        image_cx = width / 2.0
        image_cy = height / 2.0

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red_1 = np.array(
            [self.lower1_h, self.min_s, self.min_v], dtype=np.uint8
        )
        upper_red_1 = np.array([self.upper1_h, 255, 255], dtype=np.uint8)

        lower_red_2 = np.array(
            [self.lower2_h, self.min_s, self.min_v], dtype=np.uint8
        )
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
        dominance_mask = dominance_mask_bool.astype(np.uint8) * 255

        raw_red_mask = cv2.bitwise_or(hsv_mask, dominance_mask)

        kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)

        # Heading uses the minimally processed red mask so the survivor's body
        # geometry is retained. Position detection may use dilation to merge
        # small disconnected red body parts.
        orientation_mask = cv2.morphologyEx(
            raw_red_mask,
            cv2.MORPH_CLOSE,
            kernel,
            iterations=1,
        )

        detection_mask = orientation_mask.copy()

        if self.dilate_iterations > 0:
            detection_mask = cv2.dilate(
                detection_mask,
                kernel,
                iterations=self.dilate_iterations,
            )

        contours, _ = cv2.findContours(
            detection_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        hsv_pixels = int(cv2.countNonZero(hsv_mask))
        dominance_pixels = int(cv2.countNonZero(dominance_mask))
        mask_pixels = int(cv2.countNonZero(detection_mask))

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

        if mask_pixels == 0 or not contours:
            cv2.putText(
                annotated,
                "SURVIVOR: NOT DETECTED",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 0, 255),
                2,
            )
            return base_info, annotated, orientation_mask

        largest_contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(largest_contour))

        x, y, w, h = cv2.boundingRect(largest_contour)

        bbox_x = float(x)
        bbox_y = float(y)
        bbox_w = float(w)
        bbox_h = float(h)

        margin = self.heading_roi_margin_px
        roi_x0 = max(0, x - margin)
        roi_y0 = max(0, y - margin)
        roi_x1 = min(width, x + w + margin)
        roi_y1 = min(height, y + h + margin)
        roi = (roi_x0, roi_y0, roi_x1, roi_y1)

        orientation_result = self.estimate_survivor_body_axis(
            orientation_mask,
            roi,
        )

        # Position center uses the red survivor pixels when available, while
        # keeping the original detection bounding box fields unchanged.
        roi_mask = orientation_mask[roi_y0:roi_y1, roi_x0:roi_x1]
        moments = cv2.moments(roi_mask, binaryImage=True)

        if moments["m00"] > 0.0:
            cx = roi_x0 + moments["m10"] / moments["m00"]
            cy = roi_y0 + moments["m01"] / moments["m00"]
        else:
            cx = bbox_x + bbox_w / 2.0
            cy = bbox_y + bbox_h / 2.0

        if not np.isfinite(cx) or not np.isfinite(cy):
            cx = bbox_x + bbox_w / 2.0
            cy = bbox_y + bbox_h / 2.0

        orientation_valid = bool(
            area >= self.heading_min_area_px
            and orientation_result["valid"]
        )
        long_axis_angle_rad = float(
            orientation_result["long_axis_angle_rad"]
        )
        heading_error_rad = float(
            orientation_result["heading_error_rad"]
        )
        long_side_px = float(orientation_result["long_side_px"])
        short_side_px = float(orientation_result["short_side_px"])
        aspect_ratio = float(orientation_result["aspect_ratio"])
        orientation_pixels = int(
            orientation_result["orientation_pixels"]
        )
        pca_eigen_ratio = float(
            orientation_result["pca_eigen_ratio"]
        )

        detected = True
        small_target = bool(area < self.min_area)

        error_x_px = cx - image_cx
        error_y_px = cy - image_cy

        error_x_norm = clamp(error_x_px / (width / 2.0), -1.0, 1.0)
        error_y_norm = clamp(error_y_px / (height / 2.0), -1.0, 1.0)

        bearing_x_rad = math.atan(
            math.tan(self.camera_hfov_rad / 2.0) * error_x_norm
        )
        bearing_y_rad = math.atan(
            math.tan(self.camera_vfov_rad / 2.0) * error_y_norm
        )

        area_ratio = area / float(width * height)

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
            orientation_valid=orientation_valid,
            long_axis_angle_rad=long_axis_angle_rad,
            long_axis_angle_deg=math.degrees(long_axis_angle_rad),
            heading_error_rad=heading_error_rad,
            heading_error_deg=math.degrees(heading_error_rad),
            long_side_px=long_side_px,
            short_side_px=short_side_px,
            aspect_ratio=aspect_ratio,
            orientation_pixels=orientation_pixels,
            pca_eigen_ratio=pca_eigen_ratio,
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

        self.draw_detection(
            annotated,
            info,
            survivor_contour=largest_contour,
            orientation_box=orientation_result["box"],
            axis_segment=orientation_result["axis_segment"],
            orientation_center=orientation_result["center"],
        )

        return info, annotated, orientation_mask

    def draw_image_center(self, image: np.ndarray, cx: float, cy: float) -> None:
        cx_i = int(cx)
        cy_i = int(cy)

        cv2.circle(image, (cx_i, cy_i), 5, (0, 255, 255), -1)
        cv2.line(image, (cx_i - 25, cy_i), (cx_i + 25, cy_i), (0, 255, 255), 1)
        cv2.line(image, (cx_i, cy_i - 25), (cx_i, cy_i + 25), (0, 255, 255), 1)

    def draw_detection(
        self,
        image: np.ndarray,
        info: TargetNavInfo,
        survivor_contour: np.ndarray = None,
        orientation_box: np.ndarray = None,
        axis_segment: Tuple[Tuple[int, int], Tuple[int, int]] = None,
        orientation_center: np.ndarray = None,
    ) -> None:
        cx = int(round(info.center_x_px))
        cy = int(round(info.center_y_px))
        image_cx = int(round(info.image_center_x_px))
        image_cy = int(round(info.image_center_y_px))

        box_color = (0, 255, 0)
        if info.small_target:
            box_color = (0, 165, 255)

        # Keep the original axis-aligned box available only as an optional
        # position-debug overlay. It is never used for heading.
        if self.draw_axis_aligned_box:
            w = int(round(info.bbox_w))
            h = int(round(info.bbox_h))

            display_w = max(w, self.display_box_min_size)
            display_h = max(h, self.display_box_min_size)

            display_x = max(0, int(cx - display_w / 2))
            display_y = max(0, int(cy - display_h / 2))
            display_x2 = min(info.image_width - 1, display_x + display_w)
            display_y2 = min(info.image_height - 1, display_y + display_h)

            cv2.rectangle(
                image,
                (display_x, display_y),
                (display_x2, display_y2),
                box_color,
                2,
            )

        # Outline the actual detected survivor region.
        if survivor_contour is not None:
            cv2.drawContours(
                image,
                [survivor_contour],
                contourIdx=-1,
                color=(255, 255, 255),
                thickness=2,
            )

        # PCA-aligned box: this box rotates with the red survivor body axis.
        if orientation_box is not None:
            orientation_box_i = np.round(orientation_box).astype(np.int32)
            cv2.polylines(
                image,
                [orientation_box_i],
                isClosed=True,
                color=(255, 255, 0),
                thickness=2,
            )

        # Magenta arrow is the red survivor's PCA body axis.
        if axis_segment is not None:
            axis_color = (
                (255, 0, 255)
                if info.orientation_valid
                else (128, 128, 128)
            )
            cv2.arrowedLine(
                image,
                axis_segment[0],
                axis_segment[1],
                axis_color,
                3,
                tipLength=0.12,
            )

        if orientation_center is not None:
            body_center = (
                int(round(float(orientation_center[0]))),
                int(round(float(orientation_center[1]))),
            )
            cv2.circle(image, body_center, 5, (255, 0, 255), -1)

        # Blue arrow is the vehicle-forward reference projected into the image.
        reference_length = int(
            max(35.0, min(90.0, info.long_side_px * 0.55))
        )
        reference_end = (
            int(
                round(
                    cx
                    + math.sin(self.heading_reference_rad)
                    * reference_length
                )
            ),
            int(
                round(
                    cy
                    - math.cos(self.heading_reference_rad)
                    * reference_length
                )
            ),
        )
        cv2.arrowedLine(
            image,
            (cx, cy),
            reference_end,
            (255, 0, 0),
            2,
            tipLength=0.15,
        )

        cv2.circle(image, (cx, cy), 6, (0, 0, 255), -1)
        cv2.line(
            image,
            (image_cx, image_cy),
            (cx, cy),
            (0, 255, 255),
            2,
        )

        status = (
            "SURVIVOR: DETECTED SMALL"
            if info.small_target
            else "SURVIVOR: DETECTED"
        )

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
            (
                "bearing=("
                f"{math.degrees(info.bearing_x_rad):.1f}, "
                f"{math.degrees(info.bearing_y_rad):.1f}) deg"
            ),
            (20, 135),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            image,
            (
                f"area={info.area_px:.1f}, ratio={info.area_ratio:.6f}, "
                f"range_proxy={info.range_proxy:.2f}"
            ),
            (20, 165),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
        )

        orientation_text = (
            (
                f"BODY heading_err={info.heading_error_deg:+.1f}deg "
                f"axis={info.long_axis_angle_deg:+.1f}deg"
            )
            if info.orientation_valid
            else "BODY heading_err=INVALID"
        )
        cv2.putText(
            image,
            orientation_text,
            (20, 195),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            image,
            (
                f"long={info.long_side_px:.1f}px "
                f"short={info.short_side_px:.1f}px "
                f"aspect={info.aspect_ratio:.2f} "
                f"pca={info.pca_eigen_ratio:.2f} "
                f"pixels={info.orientation_pixels}"
            ),
            (20, 225),
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
            # Appended orientation fields; existing indices 0..16 remain unchanged.
            1.0 if info.orientation_valid else 0.0,
            float(info.heading_error_rad),
            float(info.heading_error_deg),
            float(info.long_axis_angle_rad),
            float(info.long_axis_angle_deg),
            float(info.long_side_px),
            float(info.short_side_px),
            float(info.aspect_ratio),
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
                f"heading={'%.2fdeg' % info.heading_error_deg if info.orientation_valid else 'INVALID'} | "
                f"aspect={info.aspect_ratio:.2f} | "
                f"pca={info.pca_eigen_ratio:.2f} pixels={info.orientation_pixels} | "
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
