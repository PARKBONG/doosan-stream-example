#!/usr/bin/env python3
"""
Doosan M1013 실시간 포지션 제어 (속도 기반)
Real-time position control for Doosan M1013 using velocity commands
"""

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from threading import Lock
import time

class RTPositionController:
    def __init__(self):
        rospy.init_node('rt_position_controller')
        
        # 현재 조인트 상태
        self.current_positions = np.zeros(6)
        self.target_positions = np.zeros(6)
        self.position_lock = Lock()
        
        # 제어 파라미터
        self.control_rate = 100  # Hz (문서에서 언급된 기본 속도)
        self.max_velocity = np.array([180, 180, 180, 350, 350, 350])  # deg/s per joint
        self.position_tolerance = 0.1  # degrees
        self.kp = 2.0  # 비례 게인
        
        # ROS 설정
        self.velocity_pub = rospy.Publisher('/dsr01m1013/speedj_rt_stream', 
                                          Float64MultiArray, queue_size=1)
        self.joint_sub = rospy.Subscriber('/dsr01m1013/joint_states', 
                                        JointState, self.joint_state_callback)
        
        # 타이머 설정
        self.control_timer = rospy.Timer(rospy.Duration(1.0/self.control_rate), 
                                       self.control_loop)
        
        rospy.loginfo("RT Position Controller initialized")
    
    def joint_state_callback(self, msg):
        """조인트 상태 업데이트"""
        with self.position_lock:
            if len(msg.position) >= 6:
                self.current_positions = np.array(msg.position[:6]) * 180.0 / np.pi  # rad to deg
    
    def set_target_position(self, target_pos):
        """목표 포지션 설정 (degrees)"""
        with self.position_lock:
            self.target_positions = np.array(target_pos)
        rospy.loginfo(f"Target position set: {target_pos}")
    
    def control_loop(self, event):
        """메인 제어 루프 (100Hz)"""
        with self.position_lock:
            current_pos = self.current_positions.copy()
            target_pos = self.target_positions.copy()
        
        # 포지션 오차 계산
        position_error = target_pos - current_pos
        
        # 각도 정규화 (-180 ~ 180)
        position_error = self.normalize_angles(position_error)
        
        # 속도 명령 계산 (비례 제어)
        velocity_cmd = self.kp * position_error
        
        # 속도 제한
        velocity_cmd = np.clip(velocity_cmd, -self.max_velocity, self.max_velocity)
        
        # 목표에 근접하면 속도를 0으로
        if np.all(np.abs(position_error) < self.position_tolerance):
            velocity_cmd = np.zeros(6)
        
        # 속도 명령 발송
        self.send_velocity_command(velocity_cmd)
    
    def normalize_angles(self, angles):
        """각도를 -180~180 범위로 정규화"""
        return ((angles + 180) % 360) - 180
    
    def send_velocity_command(self, velocities):
        """속도 명령 전송"""
        msg = Float64MultiArray()
        # speedj_rt_stream 형식: [v1, v2, v3, v4, v5, v6, time]
        msg.data = list(velocities * np.pi / 180.0)  # deg/s to rad/s
        msg.data.append(0.01)  # time parameter (10ms)
        
        self.velocity_pub.publish(msg)
    
    def move_to_position(self, target_positions, timeout=30.0):
        """특정 포지션으로 이동"""
        self.set_target_position(target_positions)
        
        start_time = time.time()
        rate = rospy.Rate(10)  # 10Hz 체크
        
        while not rospy.is_shutdown():
            with self.position_lock:
                current_pos = self.current_positions.copy()
                target_pos = self.target_positions.copy()
            
            # 도달 여부 확인
            position_error = self.normalize_angles(target_pos - current_pos)
            if np.all(np.abs(position_error) < self.position_tolerance):
                rospy.loginfo("Target position reached!")
                return True
            
            # 타임아웃 확인
            if time.time() - start_time > timeout:
                rospy.logwarn("Move timeout!")
                return False
            
            rate.sleep()
        
        return False
    
    def stop(self):
        """로봇 정지"""
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
        self.velocity_pub.publish(msg)


def main():
    try:
        controller = RTPositionController()
        
        # 실시간 제어 연결 대기
        rospy.sleep(2.0)
        
        # 예제 사용법
        rospy.loginfo("Moving to home position...")
        home_position = [0, 0, 90, 0, 90, 0]  # degrees
        success = controller.move_to_position(home_position)
        
        if success:
            rospy.loginfo("Moving to target position...")
            target_position = [30, -30, 60, 45, 60, 30]  # degrees
            controller.move_to_position(target_position)
        
        # 제어 루프 유지
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    finally:
        controller.stop()

if __name__ == '__main__':
    main()