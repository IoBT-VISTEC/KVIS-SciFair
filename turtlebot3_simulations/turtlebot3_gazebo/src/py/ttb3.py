#!/usr/bin/env python
import rospy
import math
#import numpy as np
from std_msgs.msg import String
from turtlebot3_msgs.msg import Score
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteModel

class Turtlebot3_drive():
    def __init__(self):
        self.DEG2RAD = math.pi / 180.0
        self.RAD2DEG = 180.0 / math.pi

        # Sensor's Index
        self.CENTER = 0
        self.LEFT = 1
        self.RIGHT = 2

        # Fixed Linear and Angular velocity
        self.LINEAR_VELOCITY = 0.5
        self.ANGULAR_VELOCITY = 1.5

        # TB3 States 
        self.GET_TB3_DIRECTION = 0
        self.TB3_DRIVE_FORWARD = 1
        self.TB3_RIGHT_TURN    = 2
        self.TB3_LEFT_TURN     = 3
        self.turtlebot3_state_num = 0

        # Threshold
        self.escape_range = 30.0 * self.DEG2RAD
        self.check_forward_dist = 0.7
        self.check_side_dist = 0.6

        # Remember Current and Previous Pose
        self.tb3_pose = 0.0
        self.prev_tb3_pose = 0.0

        # Distance of Sensor
        self.scan_low_data = [0.0, 0.0, 0.0]
        self.scan_high_data = [0.0, 0.0, 0.0]

        self.OBSTACLE = 0
        self.TARGET = 1
        self.obj_detect = [0, 0, 0]

        self.score = 0

        self.xbot = 0
        self.ybot = 0
        self.zbot = 0

        self.deleteModel = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel) 
        self.cmd_vel_pub = rospy.Publisher("/ttb_green/cmd_vel", Twist, queue_size=10) ##
        self.score_pub = rospy.Publisher("/score", Score, queue_size=10) ##
        self.odom_sub = rospy.Subscriber("/ttb_green/odom", Odometry, self.odomMsgCallback) ##
        self.scan_low_sub = rospy.Subscriber("/ttb_green/scan", LaserScan, self.objectScanMsgCallback) ##
        self.scan_high_sub = rospy.Subscriber("/ttb_green/scanhigh", LaserScan, self.wallScanMsgCallback) ##
        self.model_sub = rospy.Subscriber("gazebo/model_states", ModelStates, self.stateMsgCallback)

    def updatecommandVelocity(self, linear, angular):
        cmd_vel = Twist()
        cmd_vel.linear.x = linear
        cmd_vel.angular.z = angular
        self.cmd_vel_pub.publish(cmd_vel)

    def odomMsgCallback(self, msg):
        siny = 2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y)
        cosy = 1.0 - 2.0 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + msg.pose.pose.orientation.z * msg.pose.pose.orientation.z)
        self.tb3_pose = math.atan2(siny, cosy)

    def wallScanMsgCallback(self, msg):
        scan_angle = [0, 30, 330]  
        for num in range(3):
            if math.isinf(msg.ranges[scan_angle[num]]):
                self.scan_high_data[num] = msg.range_max
            else:
                self.scan_high_data[num] = msg.ranges[scan_angle[num]]

    def objectScanMsgCallback(self, msg):
        scan_angle = [0, 30, 330]  
        for num in range(3):
            if math.isinf(msg.ranges[scan_angle[num]]):
                self.scan_low_data[num] = msg.range_max
            else:
                self.scan_low_data[num] = msg.ranges[scan_angle[num]]

    def stateMsgCallback(self, msg):
        size = len(msg.pose)
        for n in range(0, size):
            if msg.name[n] == "ttb_green":
                self.xbot = msg.pose[n].position.x
                self.ybot = msg.pose[n].position.y
                self.zbot = msg.pose[n].position.z
                break

        for i in range(0, size):            
                
            xtg = msg.pose[i].position.x
            ytg = msg.pose[i].position.y
            ztg = msg.pose[i].position.z

            distance = math.sqrt(math.pow((xtg - self.xbot), 2) + math.pow((ytg - self.ybot), 2) + math.pow((ztg - self.zbot), 2))
            
            
            if distance < 0.5:
                if "beer" in msg.name[i]:
                    print("hit")
                    print(msg.name[i])
                    delM = DeleteModel()
                    print(msg.name[i])
                    #mname = String()
                    #mname.data = msg.name[i]
                    #delM.model_name = mname
                    rospy.wait_for_service('/gazebo/delete_model')
                    result = self.deleteModel(str(msg.name[i]))
                    if (result.success): 
                        self.score += 1
                        sc = Score()
                        sc.team = 'Green'
                        sc.score = self.score
                        self.score_pub.publish(sc)
                if "ttb" in msg.name[i]:
                    self.hit_ttb = True

    def controlLoop(self):
        if self.turtlebot3_state_num == self.GET_TB3_DIRECTION:
            if self.scan_high_data[self.CENTER] > self.scan_low_data[self.CENTER]: # object detected
                self.turtlebot3_state_num = self.TB3_DRIVE_FORWARD
            elif self.scan_low_data[self.LEFT] < self.scan_high_data[self.LEFT]:
                self.prev_tb3_pose = self.tb3_pose
                self.turtlebot3_state_num = self.TB3_LEFT_TURN
            elif self.scan_low_data[self.RIGHT] < self.scan_high_data[self.RIGHT]:
                self.prev_tb3_pose = self.tb3_pose
                self.turtlebot3_state_num = self.TB3_RIGHT_TURN

            if math.fabs(self.scan_low_data[self.CENTER] - self.scan_high_data[self.CENTER]) < 0.1: # wall detected
                if self.scan_high_data[self.CENTER] > self.check_forward_dist:
                    if self.scan_high_data[self.LEFT] < self.check_side_dist:
                        self.prev_tb3_pose = self.tb3_pose
                        self.turtlebot3_state_num = self.TB3_RIGHT_TURN
                    elif self.scan_high_data[self.RIGHT] < self.check_side_dist:
                        self.prev_tb3_pose = self.tb3_pose
                        self.turtlebot3_state_num = self.TB3_LEFT_TURN
                    else:
                        self.turtlebot3_state_num = self.TB3_DRIVE_FORWARD
                
                if self.scan_high_data[self.CENTER] < self.check_forward_dist:
                    self.prev_tb3_pose = self.tb3_pose
                    self.turtlebot3_state_num = self.TB3_RIGHT_TURN

        elif self.turtlebot3_state_num == self.TB3_DRIVE_FORWARD:
            print(1)
            self.updatecommandVelocity(self.LINEAR_VELOCITY, 0.0)
            self.turtlebot3_state_num = self.GET_TB3_DIRECTION

        elif self.turtlebot3_state_num == self.TB3_RIGHT_TURN:
            print(2)
            if math.fabs(self.prev_tb3_pose - self.tb3_pose) >= self.escape_range:
                self.turtlebot3_state_num = self.GET_TB3_DIRECTION
            else:
                self.updatecommandVelocity(0.0, -1 * self.ANGULAR_VELOCITY)

        elif self.turtlebot3_state_num == self.TB3_LEFT_TURN:
            print(3)
            if math.fabs(self.prev_tb3_pose - self.tb3_pose) >= self.escape_range:
                self.turtlebot3_state_num = self.GET_TB3_DIRECTION
            else:
                self.updatecommandVelocity(0.0, self.ANGULAR_VELOCITY)

        else:
            print(4)
            turtlebot3_state_num = self.GET_TB3_DIRECTION
            
        # if self.turtlebot3_state_num == self.GET_TB3_DIRECTION:
        #     print(0)
        #     if self.scan_data[self.CENTER] > self.check_forward_dist:
        #         if self.scan_data[self.LEFT] < self.check_side_dist:
        #             self.prev_tb3_pose = self.tb3_pose
        #             self.turtlebot3_state_num = self.TB3_RIGHT_TURN
        #         elif self.scan_data[self.RIGHT] < self.check_side_dist:
        #             self.prev_tb3_pose = self.tb3_pose
        #             self.turtlebot3_state_num = self.TB3_LEFT_TURN
        #         else:
        #             self.turtlebot3_state_num = self.TB3_DRIVE_FORWARD

        #     if self.scan_data[self.CENTER] < self.check_forward_dist:
        #         self.prev_tb3_pose = self.tb3_pose
        #         self.turtlebot3_state_num = self.TB3_RIGHT_TURN

        # elif self.turtlebot3_state_num == self.TB3_DRIVE_FORWARD:
        #     print(1)
        #     self.updatecommandVelocity(self.LINEAR_VELOCITY, 0.0)
        #     self.turtlebot3_state_num = self.GET_TB3_DIRECTION

        # elif self.turtlebot3_state_num == self.TB3_RIGHT_TURN:
        #     print(2)
        #     if math.fabs(self.prev_tb3_pose - self.tb3_pose) >= self.escape_range:
        #         self.turtlebot3_state_num = self.GET_TB3_DIRECTION
        #     else:
        #         self.updatecommandVelocity(0.0, -1 * self.ANGULAR_VELOCITY)

        # elif self.turtlebot3_state_num == self.TB3_LEFT_TURN:
        #     print(3)
        #     if math.fabs(self.prev_tb3_pose - self.tb3_pose) >= self.escape_range:
        #         self.turtlebot3_state_num = self.GET_TB3_DIRECTION
        #     else:
        #         self.updatecommandVelocity(0.0, self.ANGULAR_VELOCITY)

        # else:
        #     print(4)
        #     turtlebot3_state_num = self.GET_TB3_DIRECTION


if __name__ == '__main__':
    rospy.init_node('drive', anonymous=True)
    rate = rospy.Rate(10)
    try:
        ttb = Turtlebot3_drive()
        while not rospy.is_shutdown():
            ttb.controlLoop()
            rate.sleep()
    except rospy.ROSInterruptException:
        ttb.updatecommandVelocity(0.0, 0.0)
