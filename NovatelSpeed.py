#!/usr/bin/env python
import rospy
from novatel_oem7_msgs.msg import BESTVEL
from std_msgs.msg import Float64

class SpeedConverter:
    def __init__(self):
        rospy.init_node("speed_converter", anonymous=True)

        self.horizontal_speed = 0.0
        self.km_ph = 0.0
        self.sub = rospy.Subscriber("/novatel/oem7/bestvel", BESTVEL, self.bestvel_callback)
        self.pub = rospy.Publisher("/speed_kmph", Float64, queue_size=10)
    
    def bestvel_callback(self, msg):
        self.horizontal_speed = msg.hor_speed
        self.km_ph = float(self.horizontal_speed) * float(3.6)
        rospy.loginfo(f"Speed: {round(self.km_ph, 1)} kmph")
        self.pub.publish(self.km_ph)
    
    def run(self):
        rospy.spin()


if __name__ == "__main__":
    # rospy.loginfo("Start")
    speed_converter = SpeedConverter()
    speed_converter.run()