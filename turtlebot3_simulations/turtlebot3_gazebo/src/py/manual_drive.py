#!/usr/bin/env python
import rospy
import math
import random
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.srv import SetModelState

class Turtlebot3_drive():
    def __init__(self):
        self.choice = 0
        self.tb3_pose = 0.0
        self.prev_tb3_pose = 0.0
        self.turning = 0
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10) ##
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odomMsgCallback)
        self.eeg_sub = rospy.Subscriber("/chatter", String, self.callback)
        #self.model_sub = rospy.Subscriber("gazebo/model_states", ModelStates, self.stateMsgCallback)
        self.model_pub = rospy.Publisher("gazebo/set_model_state", ModelState, queue_size=10)
        self.startTime = rospy.Time.now()

        ###########################################################################
        self.orient = [
                        [0,0,math.sqrt(2)/2,math.sqrt(2)/2],
                        [0,0,1,0],
                        [0,0,0,1],
                        [0,0,-math.sqrt(2)/2,math.sqrt(2)/2],  
                      ]
        ############################################################################

    # def stateMsgCallback(self, msg):

    #     self.current_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    #     self.current_orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]

    def setPosition(self, choice):
        a = ModelState()
        a.model_name = "turtlebot3"
        a.pose.position.x = math.floor(self.current_position[0]) + 0.5
        a.pose.position.y = math.floor(self.current_position[1]) + 0.5
        a.pose.position.z = self.current_position[2] + 0.1
        if choice == 3 or choice == 4:
            randOrient = random.choice(self.orient)
            a.pose.orientation.x = randOrient[0]
            a.pose.orientation.y = randOrient[1]
            a.pose.orientation.z = randOrient[2]
            a.pose.orientation.w = randOrient[3]
        else:
            a.pose.orientation.x = self.orientation_list[0]
            a.pose.orientation.y = self.orientation_list[1]
            a.pose.orientation.z = self.orientation_list[2]
            a.pose.orientation.w = self.orientation_list[3]
        #self.model_pub(a)        

        ############################################################
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( a )

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
        ##############################################################

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
        # self.prev_orient = self.orientation_list
        if (self.choice == 1): # left
            self.prev_tb3_pose = self.tb3_pose
            while math.fabs(self.prev_tb3_pose - self.tb3_pose) < 1.57079632679:
                #print(self.tb3_pose)
                self.updatecommandVelocity(0.0, 0.1*math.pi)
            self.updatecommandVelocity(0.0, 0.0)
            # self.startTime = rospy.Time.now()
            # while rospy.Time.now() - self.startTime < rospy.Duration(5):
            #     self.updatecommandVelocity(0.2, 0)
            # self.updatecommandVelocity(0.0, 0.0)
        elif (self.choice == 2): # right
            self.prev_tb3_pose = self.tb3_pose
            while math.fabs(self.prev_tb3_pose - self.tb3_pose) < 1.57079632679:
                #print(self.tb3_pose)
                self.updatecommandVelocity(0.0, -0.1*math.pi)
            self.updatecommandVelocity(0.0, 0.0)
            # self.startTime = rospy.Time.now()
            # while rospy.Time.now() - self.startTime < rospy.Duration(5):
            #     self.updatecommandVelocity(0.2, 0)
            # self.updatecommandVelocity(0.0, 0.0)
        elif (self.choice == 4): # back
            self.startTime = rospy.Time.now()
            while rospy.Time.now() - self.startTime < rospy.Duration(5):
                self.updatecommandVelocity(-0.2, 0)
            self.updatecommandVelocity(0.0, 0.0)
        else: # forward
            self.startTime = rospy.Time.now()
            while rospy.Time.now() - self.startTime < rospy.Duration(5):
                #print(self.tb3_pose)
                self.updatecommandVelocity(0.2, -0.00000011951756)
            self.updatecommandVelocity(0.0, 0.0)
        
        self.setPosition(self.choice)

    def odomMsgCallback(self, msg):
        #siny = 2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y)
        #cosy = 1.0 - 2.0 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + msg.pose.pose.orientation.z * msg.pose.pose.orientation.z)
        #self.tb3_pose = math.atan2(siny, cosy)
        self.current_position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.orientation_list = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(self.orientation_list)
        #print(yaw)
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
