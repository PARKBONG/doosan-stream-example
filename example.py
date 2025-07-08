#!/usr/bin/env python3
import rospy
from dsr_msgs.msg import SpeedJStream, SpeedJRTStream, ServoJRTStream, ServoJStream
import numpy as np

vel = 1
acc = vel * 5
hz = 100

def generate_joint_velocity(t):
    """시간 t에 따른 joint velocity profile 예시"""
    return [vel*np.sin(t), vel*np.cos(t), 0, 0, 0, 0]
    # return [0, 0, 90, 0, 90+vel*np.sin(t), vel*np.cos(t)]

    # return [0, 0, 0, 0, vel*np.sin(t/10), vel*np.cos(t/10)]

def main():
    rospy.init_node("joint_velocity_controller")
    pub = rospy.Publisher("/dsr01m1013/speedj_stream", SpeedJStream, queue_size=10)
    # pub = rospy.Publisher("/dsr01m1013/speedj_rt_stream", SpeedJRTStream, queue_size=10)
    # pub = rospy.Publisher("/dsr01m1013/servoj_rt_stream", ServoJRTStream, queue_size=10)
    # pub = rospy.Publisher("/dsr01m1013/servoj_stream", ServoJStream, queue_size=10)
    rate = rospy.Rate(hz)  # 100 Hz

    start_time = rospy.Time.now().to_sec()
    rospy.loginfo("Publishing to /dsr01m1013/speedj_rt_stream...")

    try:
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            t = now - start_time
            print(f"Time: {t:.2f} sec")

            msg = SpeedJStream()
            # msg = SpeedJRTStream()
            # msg = ServoJRTStream()
            # msg = ServoJStream()

            # msg.pos = generate_joint_velocity(t)
            # msg.vel = [vel] * 6
            msg.vel = generate_joint_velocity(t)
            msg.acc = [acc] * 6           # 가속도 설정
            msg.time = 0                # 0.0이면 무기한 유지

            pub.publish(msg)
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Stopped publishing joint velocity commands.")

if __name__ == "__main__":
    main()