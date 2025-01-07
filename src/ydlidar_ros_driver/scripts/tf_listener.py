#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
import tf_conversions

def broadcast_transform():
    rospy.init_node('tf_broadcaster')
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    rate = rospy.Rate(10.0)  # 10 Hz
    while not rospy.is_shutdown():
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"  # 부모 프레임
        t.child_frame_id = "base_footprint"  # 자식 프레임
        t.transform.translation.x = 0.0  # 필요에 맞게 위치 설정
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)  # 필요에 맞게 회전 설정
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        br.sendTransform(t)
        rate.sleep()

if __name__ == '__main__':
    try:
        broadcast_transform()
    except rospy.ROSInterruptException:
        pass
