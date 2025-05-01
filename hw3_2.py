#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import atan2, sqrt

class MoveForward():
    def __init__(self):
        rospy.init_node('move_here')
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)

        self.start_x = None
        self.start_y = None
        self.moved = False

    def pose_callback(self, msg):
        if self.start_x is None:
            self.start_x = msg.x
            self.start_y = msg.y
            self.start_z = msg.theta  # msg.z → should be msg.theta in turtlesim
            return

        goal_x = 1
        goal_y = 2

        # Calculate angle and velocity
        goal_theta = atan2(goal_y - msg.y, goal_x - msg.x)
        v_x = sqrt((goal_x - msg.x) ** 2 + (goal_y - msg.y) ** 2)
        total_distance = sqrt((goal_x - self.start_x) ** 2 + (goal_y - self.start_y) ** 2)

        if v_x > total_distance * 0.1:
            cmd = Twist()
            cmd.linear.x = 0.1
            cmd.linear.y = 0.0
            cmd.angular.z = (goal_theta - msg.theta) * 0.5
            self.pub.publish(cmd)
        elif not self.moved:
            self.pub.publish(Twist())  # Stop
            rospy.loginfo("이동 완료")
            self.moved = True
            rospy.signal_shutdown("move here")

if __name__ == '__main__':
    try:
        MoveForward()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

