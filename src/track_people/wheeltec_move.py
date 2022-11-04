import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import time
settings = termios.tcgetattr(sys.stdin)
class Move:
    def __init__(self) -> None:
        # rospy.init_node('turtlebot_teleop')
        self.rate = rospy.Rate(50)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        pass

    def rotate(self, theta):
        if theta<0:
            angular_speed = -1.0
        else:
            angular_speed = 1.0
        angular_duration = theta / angular_speed
        ticks = int(angular_duration * 50)
        twist = Twist()
        for i in range(ticks):
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = angular_speed
            self.pub.publish(twist)
            self.rate.sleep()
        twist.angular.z = 0
        self.pub.publish(twist)
# termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)