#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Turtlebot3_drive():
    def __init__(self):
        self.choice = 0
        self.tb3_pose = 0.0
        self.prev_tb3_pose = 0.0
        self.turning = 0
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10) ##
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odomMsgCallback)
        self.eeg_sub = rospy.Subscriber("/chatter", String, self.callback)
        self.startTime = rospy.Time.now()

    def callback(self, msg):
	if msg.data == 'action':
	    return 0
        self.choice = int(msg.data)
        print(self.choice)
        self.startTime = rospy.Time.now()
        # while rospy.Time.now() - self.startTime < rospy.Duration(5):
        #     self.updatecommandVelocity(0.3, -0.0001)
        # self.updatecommandVelocity(0.0, 0.0)
        
        # while rospy.Time.now() - self.startTime < rospy.Duration(10):
        #     self.updatecommandVelocity(0.5, -0.0003874) # 90 deg
        # self.updatecommandVelocity(0.0, 0.0)

        if (self.choice == 1): # left
            self.prev_tb3_pose = self.tb3_pose
            while math.fabs(self.prev_tb3_pose - self.tb3_pose) < 1.57079632679:
                print(self.tb3_pose)
                self.updatecommandVelocity(0.0, 0.1*math.pi)
            self.updatecommandVelocity(0.0, 0.0)
            self.startTime = rospy.Time.now()
            while rospy.Time.now() - self.startTime < rospy.Duration(5):
                self.updatecommandVelocity(0.2, -0.00000011951756)
            self.updatecommandVelocity(0.0, 0.0)
        elif (self.choice == 2): # right
            self.prev_tb3_pose = self.tb3_pose
            while math.fabs(self.prev_tb3_pose - self.tb3_pose) < 1.57079632679:
                print(self.tb3_pose)
                self.updatecommandVelocity(0.0, -0.08*math.pi)
            self.updatecommandVelocity(0.0, 0.0)
            self.startTime = rospy.Time.now()
            while rospy.Time.now() - self.startTime < rospy.Duration(5):
                self.updatecommandVelocity(0.2, -0.00000011951756)
            self.updatecommandVelocity(0.0, 0.0)
        elif (self.choice == 4): # back
            self.startTime = rospy.Time.now()
            while rospy.Time.now() - self.startTime < rospy.Duration(5):
                self.updatecommandVelocity(-0.2, -0.00000011951756)
            self.updatecommandVelocity(0.0, 0.0)
        else: # forward
            self.startTime = rospy.Time.now()
            while rospy.Time.now() - self.startTime < rospy.Duration(5):
                print(self.tb3_pose)
                self.updatecommandVelocity(0.2, -0.00000011951756)
            self.updatecommandVelocity(0.0, 0.0)

    def odomMsgCallback(self, msg):
        #siny = 2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y)
        #cosy = 1.0 - 2.0 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + msg.pose.pose.orientation.z * msg.pose.pose.orientation.z)
        #self.tb3_pose = math.atan2(siny, cosy)
        orientation_list = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        print(yaw)
        self.tb3_pose = yaw

    def updatecommandVelocity(self, linear, angular):
        cmd_vel = Twist()
        cmd_vel.linear.x = linear
        cmd_vel.angular.z = angular
        self.cmd_vel_pub.publish(cmd_vel)

    #def controlLoop(self):


if __name__ == '__main__':
    rospy.init_node('eeg_drive', anonymous=True)
    ttb = Turtlebot3_drive()
    rate = rospy.Rate(10)
    rospy.spin()
    # while not rospy.is_shutdown():
    #     rate.sleep()
