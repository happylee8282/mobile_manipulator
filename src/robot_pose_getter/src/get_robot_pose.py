#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def pose_callback(msg):
    # Position coordinates (x, y, z)
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z

    # Orientation coordinates (x, y, z, w)
    ox = msg.pose.pose.orientation.x
    oy = msg.pose.pose.orientation.y
    oz = msg.pose.pose.orientation.z
    ow = msg.pose.pose.orientation.w

    # Print the position and orientation values
    rospy.loginfo("Position: x = {}, y = {}, z = {}".format(x, y, z))
    rospy.loginfo("Orientation: x = {}, y = {}, z = {}, w = {}".format(ox, oy, oz, ow))

def main():
    rospy.init_node('pose_listener', anonymous=True)
    rospy.loginfo("pose_listener node has started.")
    rospy.Subscriber('/rtabmap/localization_pose', PoseWithCovarianceStamped, pose_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
