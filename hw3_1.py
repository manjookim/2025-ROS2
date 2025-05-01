#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class MoveForward():
    def __init__(self):
        rospy.init_node('move_forward')
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)

        self.start_x = None
        self.moved = False

    def pose_callback(self, msg):
        if self.start_x is None:
            self.start_x = msg.x
            return

        if abs(msg.x - self.start_x) < 1.0:
            cmd = Twist()
            cmd.linear.x = 1  # 전진 속도
            cmd.angular.z = 0  # 회전 속도
            self.pub.publish(cmd)
        elif not self.moved:
            self.pub.publish(Twist())  # 정지
            rospy.loginfo("x축으로 1m 이동 완료")
            self.moved = True
            rospy.signal_shutdown("1m move forward")

if __name__ == '__main__':
    try:
        MoveForward()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

