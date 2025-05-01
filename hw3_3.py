#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import atan2, sqrt

class MoveHere():
    def __init__(self):
        rospy.init_node('move_here')
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)

        self.start_x = None
        self.start_y = None
        self.moved = False

        # PID gains for linear
        self.kp_lin = 1.8
        self.ki_lin = 0.15
        self.kd_lin = 0.7

        # PID gains for angular
        self.kp_ang = 2.5
        self.ki_ang = 0.15
        self.kd_ang = 0.9

        # PID state variables
        self.prev_time = rospy.Time.now()
        self.prev_lin_error = 0.0
        self.int_lin_error = 0.0

        self.prev_ang_error = 0.0
        self.int_ang_error = 0.0

    def pose_callback(self, msg):
        if self.start_x is None:
            self.start_x = msg.x
            self.start_y = msg.y
            return

        goal_x = 8.0
        goal_y = 8.0

        # 시간 계산
        curr_time = rospy.Time.now()
        dt = (curr_time - self.prev_time).to_sec()
        if dt == 0:
            return  # 나누기 0 방지

        # 위치 오차 계산
        dist_error = sqrt((goal_x - msg.x)**2 + (goal_y - msg.y)**2)
        goal_theta = atan2(goal_y - msg.y, goal_x - msg.x)
        angle_error = goal_theta - msg.theta

        # PID 계산 - Linear
        self.int_lin_error += dist_error * dt
        der_lin_error = (dist_error - self.prev_lin_error) / dt
        lin_output = (self.kp_lin * dist_error) + (self.ki_lin * self.int_lin_error) + (self.kd_lin * der_lin_error)

        # PID 계산 - Angular
        self.int_ang_error += angle_error * dt
        der_ang_error = (angle_error - self.prev_ang_error) / dt
        ang_output = (self.kp_ang * angle_error) + (self.ki_ang * self.int_ang_error) + (self.kd_ang * der_ang_error)

        # Twist 메시지 구성
        cmd = Twist()
        cmd.linear.x = min(lin_output, 1.5)  # 속도 제한
        cmd.angular.z = ang_output
        self.pub.publish(cmd)

        # 오차 상태 업데이트
        self.prev_time = curr_time
        self.prev_lin_error = dist_error
        self.prev_ang_error = angle_error

        # 목표 도달 판정
        if dist_error < 0.1 and not self.moved:
            self.pub.publish(Twist())  # 멈춤
            rospy.loginfo("PID 이동 완료")
            self.moved = True
            rospy.signal_shutdown("Done")

if __name__ == '__main__':
    try:
        MoveHere()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

