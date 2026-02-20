# Backends that produce aligned RGB + depth + camera_info for AlignedDepthImage.
# OAK uses DepthAI v3 (no XLinkOut; requestOutput/createOutputQueue; ImageAlign or stereo.inputAlignTo).
# RealSense uses rs.align(rs.stream.color).
# Both return RGB image as BGR (OpenCV convention) for consistency with YOLO.

from dataclasses import dataclass
from typing import Any, Optional

import cv2
import numpy as np


@dataclass
class AlignedFrame:
    """One aligned RGB + depth frame with intrinsics and optional hardware timestamp."""

    rgb: np.ndarray  # BGR, uint8
    depth: np.ndarray  # uint16, mm
    camera_info: Any  # sensor_msgs/CameraInfo (filled K, etc.)
    hardware_stamp_sec: int = 0
    hardware_stamp_nanosec: int = 0


def _build_camera_info_from_k(
    frame_id: str,
    height: int,
    width: int,
    k: np.ndarray,
    stamp=None,
) -> "CameraInfo":
    """Build sensor_msgs/CameraInfo from 3x3 K matrix."""
    from sensor_msgs.msg import CameraInfo

    msg = CameraInfo()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = height
    msg.width = width
    msg.distortion_model = "plumb_bob"
    msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.k = list(k.flatten())
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = list(k.flatten()) + [0.0, 0.0, 0.0]
    return msg


class OakAlignedBackend:
    """Produces aligned RGB + depth from OAK using DepthAI v3 (no XLinkOut; ImageAlign or stereo.inputAlignTo for alignment)."""

    def __init__(self, camera_key: str, cam_cfg: dict, logger=None):
        import depthai as dai

        self._key = camera_key
        self._cam_cfg = cam_cfg.get("ros__parameters", cam_cfg)
        self._logger = logger
        rgb_cfg = self._cam_cfg.get("rgb", {})
        stereo_cfg = self._cam_cfg.get("stereo", {})

        width = rgb_cfg.get("rgb_width", 640)
        height = rgb_cfg.get("rgb_height", 480)
        fps = rgb_cfg.get("rgb_fps", 30)
        stereo_size = (width, height)

        pipeline = dai.Pipeline()

        cam_rgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        stereo = pipeline.create(dai.node.StereoDepth)

        stereo.setRectification(True)
        stereo.setExtendedDisparity(stereo_cfg.get("i_extended_disp", True))
        stereo.setLeftRightCheck(True)
        try:
            stereo.setSubpixel(True)
        except Exception:
            pass

        try:
            rgb_out = cam_rgb.requestOutput(size=stereo_size, fps=fps, type=dai.ImgFrame.Type.BGR888p)
        except Exception:
            rgb_out = cam_rgb.requestOutput(size=stereo_size, fps=fps)
        left_out = left.requestOutput(size=stereo_size, fps=fps)
        right_out = right.requestOutput(size=stereo_size, fps=fps)

        left_out.link(stereo.left)
        right_out.link(stereo.right)

        try:
            align = pipeline.create(dai.node.ImageAlign)
            stereo.depth.link(align.input)
            rgb_out.link(align.inputAlignTo)
            depth_out = align.outputAligned
        except Exception:
            if hasattr(stereo, "inputAlignTo"):
                rgb_out.link(stereo.inputAlignTo)
                depth_out = stereo.depth
            else:
                raise

        self._rgb_queue = rgb_out.createOutputQueue()
        self._depth_queue = depth_out.createOutputQueue()

        pipeline.start()

        self._pipeline = pipeline
        self._width = width
        self._height = height
        self._frame_id = f"{camera_key}_color_optical_frame"

        try:
            calib = pipeline.getCalibration()
            self._k = np.array(calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, width, height))
        except Exception:
            try:
                dev = pipeline.getDefaultDevice()
                calib = dev.getCalibration()
                self._k = np.array(calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, width, height))
            except Exception:
                fx = width * 1.2
                fy = height * 1.2
                self._k = np.array([[fx, 0, width / 2.0], [0, fy, height / 2.0], [0, 0, 1.0]])

    def get_frame(self, stamp=None) -> Optional[AlignedFrame]:
        rgb_msg = self._rgb_queue.tryGet()
        depth_msg = self._depth_queue.tryGet()
        if rgb_msg is None or depth_msg is None:
            return None

        rgb = rgb_msg.getCvFrame()
        depth = depth_msg.getFrame()

        hw_sec = 0
        hw_nsec = 0
        try:
            ts = rgb_msg.getTimestamp()
            if ts is not None:
                hw_sec = getattr(ts, "seconds", 0)
                hw_nsec = getattr(ts, "nanos", 0)
        except Exception:
            pass

        camera_info = _build_camera_info_from_k(
            self._frame_id, self._height, self._width, self._k, stamp
        )
        return AlignedFrame(
            rgb=rgb,
            depth=depth,
            camera_info=camera_info,
            hardware_stamp_sec=hw_sec,
            hardware_stamp_nanosec=hw_nsec,
        )

    def shutdown(self):
        if hasattr(self, "_pipeline"):
            try:
                self._pipeline.stop()
            except Exception:
                pass


class RealsenseAlignedBackend:
    """Produces aligned RGB + depth from RealSense using rs.align(rs.stream.color)."""

    def __init__(self, camera_key: str, cam_cfg: dict, logger=None):
        import pyrealsense2 as rs

        self._key = camera_key
        self._cam_cfg = cam_cfg.get("ros__parameters", cam_cfg)
        self._logger = logger
        device_serial = self._cam_cfg.get("camera", {}).get("i_serial_number")

        self._pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 6)
        cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 6)
        if device_serial:
            cfg.enable_device(str(device_serial))
        self._profile = self._pipeline.start(cfg)

        self._align = rs.align(rs.stream.color)
        self._frame_id = f"{camera_key}_color_optical_frame"

    def get_frame(self, stamp=None) -> Optional[AlignedFrame]:
        import pyrealsense2 as rs

        frames = self._pipeline.poll_for_frames()
        if not frames:
            return None
        aligned = self._align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if color_frame is None or depth_frame is None:
            return None

        rgb = np.asanyarray(color_frame.get_data())
        rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        depth = np.asanyarray(depth_frame.get_data())

        hw_sec = 0
        hw_nsec = 0
        try:
            ts = color_frame.get_timestamp()
            if ts is not None:
                hw_sec = int(ts) // 1_000_000_000
                hw_nsec = int(ts) % 1_000_000_000
        except Exception:
            pass

        intrinsics = color_frame.get_profile().as_video_stream_profile().get_intrinsics()
        k = np.array(
            [
                [intrinsics.fx, 0, intrinsics.ppx],
                [0, intrinsics.fy, intrinsics.ppy],
                [0, 0, 1],
            ]
        )
        height = intrinsics.height
        width = intrinsics.width
        camera_info = _build_camera_info_from_k(
            self._frame_id, height, width, k, stamp
        )
        return AlignedFrame(
            rgb=rgb,
            depth=depth,
            camera_info=camera_info,
            hardware_stamp_sec=hw_sec,
            hardware_stamp_nanosec=hw_nsec,
        )

    def shutdown(self):
        if hasattr(self, "_pipeline"):
            try:
                self._pipeline.stop()
            except Exception:
                pass
