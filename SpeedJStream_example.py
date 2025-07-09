#!/usr/bin/env python3
import rospy
from dsr_msgs.msg import SpeedJStream
import numpy as np

start_velocities_deg = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# ACCELERATION SETTINGS
TOPIC_NAME = '/dsr01m1013/speedj_stream'
acc = [100] * 6
time_sec = 0.01  # Optimally 8ms for RT, 20ms for non-RT

# Sin wave parameters
AMPLITUDE_DEG = 1.5 # if larger than 1.5, the robot get 
FREQUENCY = 0.1 

class SpeedJStreamer:
    def __init__(self):
        self.pub = rospy.Publisher(TOPIC_NAME, SpeedJStream, queue_size=10)
        self.current_velocities_deg = start_velocities_deg.copy()
        self.t_start = rospy.get_time()
        rospy.Timer(rospy.Duration(time_sec), self.timer_callback)

    def timer_callback(self, event):
        t = rospy.get_time()
        vel = self.current_velocities_deg.tolist()

        msg = SpeedJStream()
        msg.vel = vel
        msg.acc = acc
        msg.time = time_sec
        self.pub.publish(msg)
        rospy.loginfo_throttle(1.0, f"Published joint velocity: {vel}")

        t = rospy.get_time()
        for i in range(6):
            self.current_velocities_deg[i] = AMPLITUDE_DEG * np.cos(2 * np.pi * (t - self.t_start) * FREQUENCY)

if __name__ == '__main__':
    rospy.init_node('doosan_speedj_publisher')
    streamer = SpeedJStreamer()
    rospy.spin()
