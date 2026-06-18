#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import PoseStamped

rospy.init_node("fixed_goal_sender")

pub = rospy.Publisher(
    "/move_base_simple/goal",
    PoseStamped,
    queue_size=1
)

rospy.sleep(2)

goal = PoseStamped()

goal.header.frame_id = "map"
goal.header.stamp = rospy.Time.now()

goal.pose.position.x = 9.158967018127441
goal.pose.position.y = 6.240289688110352
goal.pose.position.z = 0.0

goal.pose.orientation.x = 0.0
goal.pose.orientation.y = 0.0
goal.pose.orientation.z = 0.0861667448127803
goal.pose.orientation.w = 0.9962807295578737

pub.publish(goal)

rospy.loginfo("Goal sent")

rospy.sleep(1)
