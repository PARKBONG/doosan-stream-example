#!/usr/bin/env python3
import rospy
from dsr_msgs.msg import ServoJRTStream
import numpy as np

# 초기값 설정
start_angles = np.deg2rad([0, 0.0, 0.0, 0.0, 0.0, 0.0])
interval = 0.01  # 100 Hz
vel = [0.0] * 6
acc = [0.0] * 6
time_sec = 0.05  # 도달 시간

class ServoJStreamer:
    def __init__(self):
        self.pub = rospy.Publisher('/dsr01m1013/servoj_rt_stream', ServoJRTStream, queue_size=10)
        self.current_angles = start_angles.copy()
        rospy.Timer(rospy.Duration(interval), self.timer_callback)

    def timer_callback(self, event):
        msg = ServoJRTStream()
        msg.pos = self.current_angles.tolist()
        msg.vel = vel
        msg.acc = acc
        msg.time = time_sec
        self.pub.publish(msg)
        rospy.loginfo_throttle(1.0, f"Published joint pos: {np.rad2deg(self.current_angles)}")

        # (선택) 조인트 변화 로직 추가
        self.current_angles[0] += np.deg2rad(0.1) 

if __name__ == '__main__':
    rospy.init_node('servoj_rt_stream_publisher')
    streamer = ServoJStreamer()
    rospy.loginfo("Started ServoJRTStream publisher...")
    rospy.spin()