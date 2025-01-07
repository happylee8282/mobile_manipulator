#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import serial
import time

# 시리얼 포트 설정 (예: '/dev/ttyACM0'는 실제 시리얼 포트로 변경)
print("Initializing serial connection...")
ser = serial.Serial('/dev/ttyACM0', 9600)  # 포트와 보드레이트 설정
time.sleep(2)  # 시리얼 통신 초기화 대기
print("Serial connection initialized.")

connected = False  # 연결 상태 추적

def cmd_vel_callback(msg):
    global connected
    if not connected:
        print("Connected to /cmd_vel")
        connected = True  # 연결 상태 업데이트
    
    # /cmd_vel에서 linear.x와 angular.z 값을 추출
    linear_x = msg.linear.x
    angular_z = msg.angular.z
    
    # 아두이노로 전송할 문자열 데이터 생성
    data = "{:.2f},{:.2f}\n".format(linear_x, angular_z)
    ser.write(data.encode())  # 아두이노로 데이터 전송
    rospy.loginfo("Sent to Arduino: %s", data)

def main():
    # ROS 노드 초기화
    rospy.init_node('cmd_vel_to_arduino', anonymous=True)
    print("ROS node initialized.")
    
    # /cmd_vel 토픽 구독
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
    print("Subscribed to /cmd_vel")
    
    # ROS가 종료되지 않고 계속 구독할 수 있도록 spin 사용
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        ser.close()  # 시리얼 포트를 닫습니다.
