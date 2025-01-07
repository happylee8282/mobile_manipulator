#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped

# TransformBroadcaster 초기화
br = tf2_ros.TransformBroadcaster()

def pose_callback(msg):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "base_footprint"

    # 2D Pose Estimate에서 받은 x, y, yaw 값을 설정
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = 0.0

    # 방향(yaw) 설정
    t.transform.rotation = msg.pose.pose.orientation

    # 변환 발행
    br.sendTransform(t)
    rospy.loginfo(f"Updated position: x={t.transform.translation.x}, y={t.transform.translation.y}")

def main():
    rospy.init_node('dynamic_tf_broadcaster')

    # /initialpose 토픽 구독
    rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, pose_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
