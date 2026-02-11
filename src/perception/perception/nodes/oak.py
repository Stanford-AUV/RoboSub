#!/usr/bin/env python3
import depthai as dai
import rclpy
from sensor_msgs.msg import Image
import os
import time
from .generic_camera import GenericCameraNode

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
}


def config_rgb_image(pipeline, key, cam_cfg):
    rgb_cfg = cam_cfg.get("rgb")

    width = rgb_cfg.get("rgb_width", 640)
    height = rgb_cfg.get("rgb_height", 480)
    fps = rgb_cfg.get("rgb_fps", 30)

    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setPreviewSize(width, height)
    cam_rgb.setFps(fps)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    video_queue = cam_rgb.video.createOutputQueue()
    return video_queue


def config_stereo_image(pipeline, key, cam_cfg):
    mono_left = pipeline.create(dai.node.MonoCamera)
    mono_right = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)

    stereo_cfg = cam_cfg.get("stereo")

    stereo.setDefaultProfilePreset(preset_map[stereo_cfg.get("i_depth_preset")])
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(True)

    stereo.setExtendedDisparity(stereo_cfg.get("i_extended_disp", False))

    mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
    mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

    mono_left.setResolution(resolution_map[stereo_cfg["left"]["i_resolution"]])
    mono_right.setResolution(resolution_map[stereo_cfg["right"]["i_resolution"]])

    mono_left.out.link(stereo.left)
    mono_right.out.link(stereo.right)

    depth_queue = stereo.depth.createOutputQueue()
    return depth_queue


class OakNode(GenericCameraNode):
    def __init__(self, path):
        super().__init__("oak_node", path)

        self.path = path
        self._camera_info = self.get_cam_data(self.load_cameras_yaml(), "oak")

        for key, cam_cfg in self._camera_info.items():
            cam_cfg = cam_cfg.get("ros__parameters")

            self.get_logger().info(
                f"Starting OAK camera: {key} (mxid={cam_cfg['camera']['i_mx_id']})"
            )

            rgb_pub = self.create_publisher(Image, f"/camera/{key}/rgb", 10)
            self.rgb_pubs[key] = rgb_pub

            depth_pub = self.create_publisher(Image, f"/camera/{key}/depth", 10)
            self.depth_pubs[key] = depth_pub

            device = self._open_device_with_retry(cam_cfg["camera"]["i_mx_id"])
            pipeline = dai.Pipeline(device)

            rgb = config_rgb_image(pipeline, key, cam_cfg)
            depth = config_stereo_image(pipeline, key, cam_cfg)

            self.devices[key] = pipeline
            self.queues[key] = {
                "rgb": rgb,
                "depth": depth,
            }

            pipeline.start()

        self.create_timer(1.0 / 30.0, self.publish_frames)  # 30 FPS

    def _open_device_with_retry(self, mxid, max_attempts=5, delay_s=1.0):
        last_exc = None
        for attempt in range(1, max_attempts + 1):
            try:
                self.get_logger().info(
                    f"Connecting to OAK device (mxid={mxid}), attempt {attempt}/{max_attempts}"
                )
                device_info = self._wait_for_device_info(mxid, delay_s)
                return dai.Device(device_info)
            except RuntimeError as exc:
                last_exc = exc
                available = self._get_available_mxids()
                available_str = ", ".join(available) if available else "none"
                self.get_logger().warning(
                    f"Failed to open OAK device (mxid={mxid}) on attempt {attempt}/{max_attempts}.\n Available devices: {available_str}. Error: {exc}",
                )
                if attempt < max_attempts:
                    time.sleep(delay_s)
        raise last_exc

    def _wait_for_device_info(self, mxid, delay_s):
        devices = dai.Device.getAllAvailableDevices()
        for device in devices:
            if device.getDeviceId() == mxid:
                return device
        time.sleep(delay_s)
        return dai.DeviceInfo(mxid)

    @staticmethod
    def _get_available_mxids():
        try:
            return [device.getMxId() for device in dai.Device.getAllAvailableDevices()]
        except Exception:
            return []

    def publish_frames(self):
        for key in self.devices.keys():
            rgb_frame = self.queues[key]["rgb"].tryGet()
            depth_frame = self.queues[key]["depth"].tryGet()
            timestamp = self.get_clock().now().to_msg()

            if rgb_frame is not None:
                rgb = rgb_frame.getCvFrame()
                msg = self.bridge.cv2_to_imgmsg(rgb, "bgr8")
                msg.header.stamp = timestamp
                self.rgb_pubs[key].publish(msg)

            if depth_frame is not None:
                depth = depth_frame.getFrame()
                msg = self.bridge.cv2_to_imgmsg(depth, "16UC1")
                msg.header.stamp = timestamp
                self.depth_pubs[key].publish(msg)


def main(args=None):
    rclpy.init(args=args)

    SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
    yaml_path = os.path.join(SCRIPT_DIR, "..", "cameras.yaml")
    yaml_path = os.path.abspath(yaml_path)

    node = OakNode(yaml_path)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
