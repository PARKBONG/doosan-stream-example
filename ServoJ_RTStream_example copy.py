#!/usr/bin/env python3
import rospy
from dsr_msgs.msg import ServoJRTStream

def rt_servoj_callback(msg: ServoJRTStream):
    pos = [round(p, 3) for p in msg.pos]
    vel = [round(v, 3) for v in msg.vel]
    acc = [round(a, 3) for a in msg.acc]
    time = round(msg.time, 3)

    rospy.loginfo_throttle(1.0, f"[RT STATE] pos: {pos}, vel: {vel}, acc: {acc}, time: {time}")

def main():
    rospy.init_node('doosan_rt_servoj_listener')
    rospy.Subscriber('/dsr01m1013/rt_servoj_stream', ServoJRTStream, rt_servoj_callback)
    rospy.loginfo("RT servoj subscriber initialized.")
    rospy.spin()

if __name__ == '__main__':
    main()