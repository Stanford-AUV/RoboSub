#!/usr/bin/env python3

import argparse
import depthai as dai
import cv2
import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst

Gst.init(None)

def gstreamer_pipeline_udp(host="192.168.1.100", port=5000):
    """
    Creates a GStreamer pipeline that expects raw BGR frames on appsrc,
    encodes them, and sends via UDP to specified host/port.
    """
    pipeline_str = f"""
        appsrc name=src is-live=true block=true format=GST_FORMAT_TIME caps=video/x-raw,format=BGR,width=1280,height=720,framerate=30/1 !
        videoconvert !
        x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast !
        rtph264pay !
        udpsink host={host} port={port}
    """
    return pipeline_str

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Stream frames from OAK-D over UDP to a remote host.")
    parser.add_argument("--host_ip", type=str, default="10.242.240.251",
                        help="IP of the machine receiving the stream (default selena's work mac: 10.242.240.251)")
    parser.add_argument("--port", type=int, default=5000,
                        help="UDP port to send the stream on (default: 5000)")
    args = parser.parse_args()

    # Create pipeline for DepthAI
    depthai_pipeline = dai.Pipeline()
    cam_rgb = depthai_pipeline.createColorCamera()
    cam_rgb.setPreviewSize(1280, 720)
    cam_rgb.setInterleaved(False)
    cam_rgb.setFps(30)

    xout_video = depthai_pipeline.createXLinkOut()
    xout_video.setStreamName("video")
    cam_rgb.preview.link(xout_video.input)

    # Create GStreamer Pipeline
    gst_pipeline_str = gstreamer_pipeline_udp(host=args.host_ip, port=args.port)
    gst_pipeline = Gst.parse_launch(gst_pipeline_str)
    appsrc = gst_pipeline.get_by_name("src")

    gst_pipeline.set_state(Gst.State.PLAYING)

    with dai.Device(depthai_pipeline) as device:
        q_video = device.getOutputQueue(name="video", maxSize=4, blocking=False)

        while True:
            in_frame = q_video.tryGet()
            if in_frame is not None:
                frame = in_frame.getCvFrame()
                # Convert the OpenCV frame (numpy array) into a Gst.Buffer
                data = frame.tobytes()
                buf = Gst.Buffer.new_allocate(None, len(data), None)
                buf.fill(0, data)
                
                # Set timestamp and duration (helps GStreamer maintain timing)
                timestamp = Gst.util_uint64_scale(
                    int(cv2.getTickCount()),
                    Gst.SECOND,
                    int(cv2.getTickFrequency())
                )
                buf.pts = buf.dts = timestamp
                buf.duration = Gst.util_uint64_scale(1, Gst.SECOND, 30)

                # Push buffer into the pipeline
                appsrc.emit("push-buffer", buf)
            else:
                # If no frame is available, continue
                pass

if __name__ == "__main__":
    main()

