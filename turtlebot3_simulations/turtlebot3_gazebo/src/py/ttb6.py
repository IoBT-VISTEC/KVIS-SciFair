#!/usr/bin/env python
from turtlebot_drive import Turtlebot3_drive
import rospy
import random

class drive(Turtlebot3_drive):

    def __init__(self, team):
        super(drive, self).__init__(team)

    def logic(self):
        if self.initial:
            self.initial = False
            return "walk"
        '''
        This TTB will random opertaion from choice. But if operation is "run", it will run for 5 steps then random new operation
        '''
        operation_choice = ["run","stop","turn left","turn right"]

        operation = random.choice(operation_choice)

        is_moving = self.mem[0]

        center_dist = self.top_center_sensor - self.bottom_center_sensor

        if center_dist < 0.1:
            if self.current_vel != 0:
                return "stop"
            else:
                return "turn right"

        if is_moving < 5: # TTB runs 5 steps 
            is_moving += 1
            self.mem[0] += 1
            return "run"
        else:
            is_moving = 0
            self.mem[0] = 0
            return operation

if __name__ == '__main__':
    rospy.init_node('drive', anonymous=True)
    rate = rospy.Rate(10)
    try:
        ttb = drive("red")
        while not rospy.is_shutdown():
            ttb.controlLoop()
            rate.sleep()
    except rospy.ROSInterruptException:
        ttb.updatecommandVelocity(0.0, 0.0)
