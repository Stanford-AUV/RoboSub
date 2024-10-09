#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import cv2
import os
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

class ImagePublisher(Node):
    def __init__(self, file_path):
        super().__init__("image_publisher")
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(file_path)
        self.pub = self.create_publisher(Image, "oak/rgb/image_raw", 10)

    def run(self):
        count = 0
        while(self.cap.isOpened()):
            ret, frame = self.cap.read() 
            if ret:
                if count % 100 == 0:
                    print(count)
                    self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
                    time.sleep(6)
                count += 1
            else:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                count = 0


        self.cap.release()

def main(args=None):
    rclpy.init(args=args)

    if(len(sys.argv) != 2):
        print("Incorrect number of arguments\nUsage:\n\tpython3 <path_to_video_file>")
        exit()

    if not os.path.isfile(sys.argv[1]):
        print("Invalid file path")
        exit()

    ip = ImagePublisher(sys.argv[1])
    print("Publishing...")
    ip.run()

    ip.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
