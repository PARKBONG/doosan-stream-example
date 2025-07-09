#!/usr/bin/env python3
import rospy
from dsr_msgs.msg import ServoJRTStream
import numpy as np
start_angles_deg = np.array([0.0, 0.0, 90.0, 0.0, 90.0, 0.0])

# VELOCITY AND ACCELERATION SETTINGS
TOPIC_NAME = '/dsr01m1013/rt_servoj_stream'
vel = [100] * 6
acc = [100] * 6
time_sec = 0.01  # Optimally 8ms for RT, 20ms for non-RT

# Sin wave parameters
AMPLITUDE_DEG = 2.5 # if larger than 2.5, the robot get joint lock
FREQUENCY = 0.1 


class ServoJStreamer:
    def __init__(self):

        self.pub = rospy.Publisher(TOPIC_NAME, ServoJRTStream, queue_size=10)
        self.current_angles_deg = start_angles_deg.copy()
        self.t_start = rospy.get_time()
        rospy.Timer(rospy.Duration(time_sec), self.timer_callback)

    def timer_callback(self, event):

        msg = ServoJRTStream()
        msg.pos = self.current_angles_deg.tolist()
        msg.vel = vel
        msg.acc = acc
        msg.time = time_sec
        self.pub.publish(msg)
        rospy.loginfo_throttle(1.0, f"Published joint pos: {self.current_angles_deg}")

        t = rospy.get_time()
        for i in range(6):
            self.current_angles_deg[i] = start_angles_deg[i] + AMPLITUDE_DEG * np.sin(2 * np.pi * (t - self.t_start) * FREQUENCY)

if __name__ == '__main__':
    rospy.init_node('doosan_streamj_publisher')
    streamer = ServoJStreamer()
    rospy.spin()