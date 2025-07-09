#!/usr/bin/env python3
import rospy
from dsr_msgs.msg import ServoJRTStream, ServoJStream
import numpy as np

start_angles_deg = np.array([0.0, 0.0, 90.0, 0.0, 90.0, 0.0])
vel = [100] * 6
acc = [100] * 6
time_sec = 0.008  # Optimally 8ms for RT, 20ms for non-RT

AMPLITUDE = 1 # DEGREE
USE_RT = True
class ServoJStreamer:
    def __init__(self):

        if USE_RT:
            self.pub = rospy.Publisher('/dsr01m1013/servoj_rt_stream', ServoJRTStream, queue_size=10)
        else:
            self.pub = rospy.Publisher('/dsr01m1013/servoj_stream', ServoJStream, queue_size=10)

        self.current_angles_deg = start_angles_deg.copy()
        self.t_start = rospy.get_time()
        rospy.Timer(rospy.Duration(time_sec), self.timer_callback)

    def timer_callback(self, event):

        if USE_RT:
            msg = ServoJRTStream()
            msg.pos = self.current_angles_deg.tolist()
            msg.vel = vel
            msg.acc = acc
            msg.time = time_sec
            self.pub.publish(msg)
            rospy.loginfo_throttle(1.0, f"[RT] Published joint pos: {self.current_angles_deg}")

        else:
            msg = ServoJStream()
            msg.pos = self.current_angles_deg.tolist()
            msg.vel = vel
            msg.acc = acc
            msg.time = time_sec
            self.pub.publish(msg)
            rospy.loginfo_throttle(1.0, f"Published joint pos: {self.current_angles_deg}")

        t = rospy.get_time()
        for i in range(6):
            self.current_angles_deg[i] = start_angles_deg[i] + AMPLITUDE * np.sin(2 * np.pi * (t - self.t_start) / 10)


if __name__ == '__main__':
    rospy.init_node('doosan_streamj_publisher')
    streamer = ServoJStreamer()
    rospy.loginfo("Started  doosan_streamj_publisher...")
    rospy.spin()