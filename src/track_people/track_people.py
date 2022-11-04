#!/usr/bin/python3
import rospy
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
import actionlib
from actionlib_msgs.msg import GoalStatus
import numpy as np
import wheeltec_move
from pid import PID
class FollowMe:
    def __init__(self) -> None:
        rospy.init_node('people_tf_listener',anonymous=True)
        self.SPEED_PUB = rospy.Publisher("cmd_vel", Twist)
        self.pid_controller = PID()
        self.pid_controller.setSampleTime(1 / 5.0)
        self.max_linear_speed = 0.8
        self.max_angle_speed = 1.0
        self.listener = tf.TransformListener()
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move = wheeltec_move.Move()
        self.rate = rospy.Rate(5.0)
        pass

    def get_tf(self):
        # transform = self.listener.allFramesAsString()
        while not rospy.is_shutdown():
            try:
                t = self.listener.getLatestCommonTime("kinect2_down_link", 'person1.0')
                trans, rot = self.listener.lookupTransform("kinect2_down_link", 'person1.0', t)
                # self.pose, self.rot = self.listener.lookupTransform("map", 'base_link', t)
                # (r, p, self.y) = tf.transformations.euler_from_quaternion(rot)4
                self.distance = np.sqrt(trans[0]**2+trans[1]**2)
                self.yaw = np.arctan(trans[0]/trans[2])
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
    def publish_move_goal(self):
        # rospy.loginfo('sync: distance={}, yaw={}'.format(self.distance, self.yaw))
        speed = Twist()
        self.pid_controller.update(self.yaw)
        output = self.pid_controller.output
        if output > self.max_angle_speed:
            output = self.max_angle_speed
        
        speed.angular.z = output
        speed.linear.x = self.max_linear_speed*np.exp(-1/self.distance)
        print(output, speed.linear.x)
        # self.SPEED_PUB.publish(speed)
        # self.move.rotate(-self.yaw)
        # goal = MoveBaseGoal()
        # goal.target_pose.header.frame_id = 'map'
        # goal.target_pose.header.stamp = rospy.Time.now()
        # goal.target_pose.pose = Pose(Point((self.trans[0]+self.pose[0])/2,(self.pose[1]+self.trans[1])/2,0), Quaternion(0,0,np.cos(self.theta),np.sin(self.theta)))
        # # Start moving
        # self.move_base.send_goal(goal)
        # success = self.move_base.wait_for_result(rospy.Duration(300))

if __name__ == '__main__':
    follow = FollowMe()
    while True:
        follow.get_tf()
        follow.publish_move_goal()
        follow.rate.sleep()
        