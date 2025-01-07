#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

def pose_callback(msg):
    # Position coordinates (x, y, z)
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z

    # Orientation as quaternion (x, y, z, w)
    ox = msg.pose.orientation.x
    oy = msg.pose.orientation.y
    oz = msg.pose.orientation.z
    ow = msg.pose.orientation.w

    # Print or process the coordinates
    rospy.loginfo("Position: x = %f, y = %f, z = %f", x, y, z)
    rospy.loginfo("Orientation: x = %f, y = %f, z = %f, w = %f", ox, oy, oz, ow)

def main():
    rospy.init_node('pose_listener', anonymous=True)
    rospy.Subscriber('/slam_out_pose', PoseStamped, pose_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
