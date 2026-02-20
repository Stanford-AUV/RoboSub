# Backends that produce aligned RGB + depth + camera_info for AlignedDepthImage.
# OAK uses DepthAI with setDepthAlign(CAM_A); RealSense uses rs.align(rs.stream.color).
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
    """Produces aligned RGB + depth from OAK using DepthAI with setDepthAlign(CAM_A)."""

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

        resolution_map = {
            "400P": dai.MonoCameraProperties.SensorResolution.THE_400_P,
            "480P": dai.MonoCameraProperties.SensorResolution.THE_480_P,
            "720P": dai.MonoCameraProperties.SensorResolution.THE_720_P,
            "800P": dai.MonoCameraProperties.SensorResolution.THE_800_P,
            "1080P": dai.MonoCameraProperties.SensorResolution.THE_1200_P,
        }
        preset_map = {
            "FAST_ACCURACY": dai.node.StereoDepth.PresetMode.FAST_ACCURACY,
            "FAST_DENSITY": dai.node.StereoDepth.PresetMode.FAST_DENSITY,
            "HIGH_ACCURACY": dai.node.StereoDepth.PresetMode.HIGH_ACCURACY,
            "HIGH_DENSITY": dai.node.StereoDepth.PresetMode.HIGH_DENSITY,
        }

        pipeline = dai.Pipeline()
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(width, height)
        cam_rgb.setFps(fps)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_right = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)

        preset = stereo_cfg.get("i_depth_preset", "FAST_ACCURACY")
        stereo.setDefaultProfilePreset(preset_map.get(preset, dai.node.StereoDepth.PresetMode.FAST_ACCURACY))
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        stereo.setExtendedDisparity(stereo_cfg.get("i_extended_disp", False))
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(width, height)

        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        left_res = stereo_cfg.get("left", {}).get("i_resolution", "400P")
        right_res = stereo_cfg.get("right", {}).get("i_resolution", "400P")
        mono_left.setResolution(resolution_map.get(left_res, dai.MonoCameraProperties.SensorResolution.THE_400_P))
        mono_right.setResolution(resolution_map.get(right_res, dai.MonoCameraProperties.SensorResolution.THE_400_P))

        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        xout_depth.setStreamName("depth")
        cam_rgb.preview.link(xout_rgb.input)
        stereo.depth.link(xout_depth.input)

        mxid = self._cam_cfg.get("camera", {}).get("i_mx_id")
        if mxid:
            device_info = None
            for d in dai.Device.getAllAvailableDevices():
                if d.getMxId() == mxid or d.getDeviceId() == mxid:
                    device_info = d
                    break
            if device_info is None:
                device_info = dai.DeviceInfo(mxid)
            self._device = dai.Device(pipeline, device_info)
        else:
            self._device = dai.Device(pipeline)

        self._calib = self._device.readCalibration()
        self._k = np.array(self._calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, width, height))
        self._width = width
        self._height = height
        self._frame_id = f"{camera_key}_color_optical_frame"

        self._rgb_queue = self._device.getOutputQueue("rgb", maxSize=4, blocking=False)
        self._depth_queue = self._device.getOutputQueue("depth", maxSize=4, blocking=False)

    def get_frame(self, stamp=None) -> Optional[AlignedFrame]:
        import depthai as dai

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
                hw_sec = ts.seconds
                hw_nsec = ts.nanos
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
        if hasattr(self, "_device"):
            self._device.close()


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
