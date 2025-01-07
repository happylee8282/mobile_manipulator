#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path

def path_callback(msg):
    path_coordinates = []
    for pose in msg.poses:
        x = pose.pose.position.x
        y = pose.pose.position.y
        path_coordinates.append((x, y))
    print("경로 좌표들:", path_coordinates)
    
    # 첫 콜백 실행 후 노드를 종료하여 한 번만 출력
    rospy.signal_shutdown("One-time path output")

def main():
    rospy.init_node('path_listener')
    rospy.Subscriber('/move_base/NavfnROS/plan', Path, path_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
