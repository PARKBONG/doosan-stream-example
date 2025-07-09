#!/usr/bin/env python3
import rospy
from dsr_msgs.msg import ServoLStream
import numpy as np

# Initial pose of the end effector in Cartesian space (x, y, z, Z, Y, Z) in mm and degrees
# this is a good starting point for the Doosan M1013 robot
start_pose = np.array([550.0, 30.0, 600.0, 0.0, 180.0, 0.0]) # xyzZYZ # this is very ugly representation, since Y=+-180 causes Jimbal lock. but no way.


# VELOCITY AND ACCELERATION SETTINGS
TOPIC_NAME = '/dsr01m1013/servol_stream'
vel = [100, 200]  # linear velocity, angular velocity for Cartesian space
acc = [100, 200]  # linear acceleration, angular acceleration for Cartesian space
time_sec = 0.01

# Sin wave parameters
AMPLITUDE_MM = 5
FREQUENCY_MM = 1/10

AMPLITUDE_DEG = 1
FREQUENCY_DEG = 1/10

class ServoLStreamer:
    def __init__(self):

        self.pub = rospy.Publisher(TOPIC_NAME, ServoLStream, queue_size=10)
        self.current_pose = start_pose.copy()
        self.t_start = rospy.get_time()
        rospy.Timer(rospy.Duration(time_sec), self.timer_callback)

    def timer_callback(self, event):

        msg = ServoLStream()
        msg.pos = self.current_pose.tolist()
        msg.vel = vel
        msg.acc = acc
        msg.time = time_sec
        self.pub.publish(msg)
        rospy.loginfo_throttle(1.0, f"Published ee pos: {self.current_pose}")

        t = rospy.get_time()
        dx = AMPLITUDE_MM * np.sin(2 * np.pi * (t - self.t_start) * FREQUENCY_MM)
        dr = AMPLITUDE_DEG * np.sin(2 * np.pi * (t - self.t_start) * FREQUENCY_DEG)
        # self.current_pose[:3] = start_pose[:3] + dx
        self.current_pose[3:] = start_pose[3:] + dr

if __name__ == '__main__':
    rospy.init_node('doosan_streaml_publisher')
    streamer = ServoLStreamer()
    rospy.spin()