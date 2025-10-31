# file: gz_camera_stream_direct.py
import threading
import time
import numpy as np
import cv2
from flask import Flask, Response
import gz.transport13 as gz
from gz.msgs10 import image_pb2

CAM_TOPIC = "/camera/image"

app = Flask(__name__)
frame_lock = threading.Lock()
latest_frame = None

def gz_image_to_cv2(msg: image_pb2.Image):
    """Convert Gazebo Image protobuf to OpenCV BGR"""
    width = msg.width
    height = msg.height
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

def camera_callback(proto_bytes, _):
    global latest_frame
    try:
        gz_img = image_pb2.Image()
        gz_img.ParseFromString(proto_bytes)
        frame = gz_image_to_cv2(gz_img)
        with frame_lock:
            latest_frame = frame
    except Exception as e:
        print(f"[ERROR] camera_callback: {e}")

# Flask MJPEG stream
def generate():
    global latest_frame
    while True:
        with frame_lock:
            if latest_frame is None:
                continue
            ret, jpeg = cv2.imencode(".jpg", latest_frame)
            if not ret:
                continue
            frame_bytes = jpeg.tobytes()
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n")

@app.route("/cam")
def cam():
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

def run_flask():
    app.run(host="0.0.0.0", port=5001, threaded=True)

if __name__ == "__main__":
    # Start Flask server in a separate thread
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()

    # Gazebo node
    node = gz.Node()
    node.subscribe_raw(
        CAM_TOPIC,
        camera_callback,
        image_pb2.Image.DESCRIPTOR.full_name,
        gz.SubscribeOptions()
    )

    print(f"Subscribed to Gazebo topic {CAM_TOPIC}")

    # Keep processing Gazebo messages
    while True:
        time.sleep(0.001)
