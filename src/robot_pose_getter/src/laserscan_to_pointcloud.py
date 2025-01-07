#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection

def scan_callback(scan_msg):
    # LaserScan 메시지를 PointCloud2로 변환
    cloud_msg = projector.projectLaser(scan_msg)
    # 변환한 PointCloud2 메시지를 발행
    pc_pub.publish(cloud_msg)

if __name__ == "__main__":
    rospy.init_node("laserscan_to_pointcloud")

    # LaserProjection 객체 생성
    projector = LaserProjection()

    # 구독 및 발행 설정
    scan_sub = rospy.Subscriber("/scan", LaserScan, scan_callback)
    pc_pub = rospy.Publisher("/scan_pointcloud", PointCloud2, queue_size=1)

    rospy.spin()
