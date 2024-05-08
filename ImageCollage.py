#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
import cv_bridge
import time

class ImageCollagePublisher:
    def __init__(self, topics, output_topic, default_image_path, timeout=0.03):
        rospy.init_node('image_collage_publisher', anonymous=True)
        self.bridge = cv_bridge.CvBridge()
        self.default_image = default_image_path
        self.default_image = cv2.resize(self.default_image, (320, 180), interpolation=cv2.INTER_AREA)
        self.images = [self.default_image.copy() for _ in topics]
        self.last_received = [time.time()] * len(topics)
        self.timeout = timeout
        self.publisher = rospy.Publisher(output_topic, CompressedImage, queue_size=1)
        self.subscribers = [rospy.Subscriber(topic, CompressedImage, self.callback, callback_args=index) for index, topic in enumerate(topics)]
        self.check_timer = rospy.Timer(rospy.Duration(1), self.check_stale_images)

    def callback(self, data, index):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data)
            cv_image = cv2.resize(cv_image, (320, 180), interpolation=cv2.INTER_AREA)
            self.images[index] = cv_image
            self.last_received[index] = time.time()
            rospy.loginfo(f"Image from topic index {index} updated and resized.")
        except cv_bridge.CvBridgeError as e:
            rospy.logerr(f"Could not convert image from topic index {index}: {str(e)}")
        self.publish_collage()

    def check_stale_images(self, event):
        current_time = time.time()
        for i, last_time in enumerate(self.last_received):
            if current_time - last_time > self.timeout:
                self.images[i] = self.default_image.copy()
                rospy.logwarn(f"No new image from topic index {i}, using default image.")

    def publish_collage(self):
        top = np.hstack(self.images[:3])
        bottom = np.hstack((self.images[3:]))
        collage = np.vstack((top, bottom))
        try:
            msg = self.bridge.cv2_to_compressed_imgmsg(collage)
            self.publisher.publish(msg)
            rospy.loginfo("Collage published.")
        except cv_bridge.CvBridgeError as e:
            rospy.logerr(f"Could not convert collage to compressed message: {str(e)}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    topics = [
        "/clpe_ros/cams_0/image_raw/compressed",
        "/clpe_ros/cams_1/image_raw/compressed",
        "/clpe_ros/cams_2/image_raw/compressed",
        "/clpe_ros/cams_3/image_raw/compressed",
        "/clpe_ros/cams_2/image_raw/compressed",
        "/clpe_ros/cams_3/image_raw/compressed",
        # "/inji1_cam/cam_0/image_raw/compressed",
        # "/inji1_cam/cam_1/image_raw/compressed",
        # "/inji1_cam/cam_2/image_raw/compressed",
        # "/inji1_cam/cam_3/image_raw/compressed",
        # "/inji2_cam/cam_2/image_raw/compressed",
        # "/inji2_cam/cam_3/image_raw/compressed",
    ]
    output_topic = "/image_collage/compressed"
    default_image_path = cv2.imread("default.jpg")
    collage_publisher = ImageCollagePublisher(topics, output_topic, default_image_path)
    collage_publisher.run()
