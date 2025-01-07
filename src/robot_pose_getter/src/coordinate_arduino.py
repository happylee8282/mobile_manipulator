#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import serial
import time

# 시리얼 포트 설정 (예: '/dev/ttyACM0'는 실제 시리얼 포트로 변경)
print("Initializing serial connection...")
ser = serial.Serial('/dev/ttyACM0', 9600)  # 포트와 보드레이트 설정
time.sleep(2)  # 시리얼 통신 초기화 대기
print("Serial connection initialized.")

connected_pose = False  # pose 연결 상태 추적

def pose_callback(msg):
    global connected_pose
    if not connected_pose:
        print("Connected to /slam_out_pose")
        connected_pose = True  # 연결 상태 업데이트
    
    # Position coordinates (x, y, z)
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    
    # 아두이노로 전송할 좌표 데이터 생성
    data = "POS,{:.2f},{:.2f},{:.2f}\n".format(x, y, z)
    ser.write(data.encode())  # 아두이노로 데이터 전송
    rospy.loginfo("Sent position to Arduino: %s", data)

def main():
    # ROS 노드 초기화
    rospy.init_node('pose_to_arduino', anonymous=True)
    print("ROS node initialized.")
    
    # /slam_out_pose 토픽 구독
    rospy.Subscriber("/slam_out_pose", PoseStamped, pose_callback)
    print("Subscribed to /slam_out_pose")
    
    # ROS가 종료되지 않고 계속 구독할 수 있도록 spin 사용
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        ser.close()  # 시리얼 포트를 닫습니다.
