#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
import math
import time

def simple_rt_test():
    rospy.init_node('simple_rt_test')
    
    # 퍼블리셔 생성
    pub = rospy.Publisher('/dsr01m1013/speedj_rt_stream', 
                         Float64MultiArray, queue_size=1)
    
    rospy.sleep(1.0)  # 초기화 대기
    
    rate = rospy.Rate(100)  # 100Hz
    start_time = time.time()
    
    rospy.loginfo("Starting simple RT test - Joint 6 will move slowly")
    
    while not rospy.is_shutdown() and (time.time() - start_time) < 10.0:
        msg = Float64MultiArray()
        
        # 10초 동안 6번째 조인트만 천천히 회전
        elapsed = time.time() - start_time
        if elapsed < 5.0:
            joint6_vel = 0.1  # 매우 느린 속도 (rad/s)
        else:
            joint6_vel = -0.1  # 반대 방향
        
        # [joint1_vel, joint2_vel, ..., joint6_vel, time]
        msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, joint6_vel, 0.01]
        
        pub.publish(msg)
        rate.sleep()
    
    # 정지
    msg = Float64MultiArray()
    msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
    pub.publish(msg)
    
    rospy.loginfo("RT test completed")

if __name__ == '__main__':
    try:
        simple_rt_test()
    except rospy.ROSInterruptException:
        pass
