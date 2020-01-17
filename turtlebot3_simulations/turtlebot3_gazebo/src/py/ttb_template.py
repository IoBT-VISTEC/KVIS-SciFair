from turtlebot_drive import Turtlebot3_drive
import rospy
import random

class drive(Turtlebot3_drive):

    def logic(self):
        if self.initial:
            self.initial = False
            return "walk"
        
        # Write your code here

if __name__ == '__main__':
    rospy.init_node('drive', anonymous=True)
    rate = rospy.Rate(10)
    try:
        ttb = drive("yellow") # change TTB color "black","blue","green","orange","purple","red","white","yellow"
        while not rospy.is_shutdown():
            ttb.controlLoop()
            rate.sleep()
    except rospy.ROSInterruptException:
        ttb.updatecommandVelocity(0.0, 0.0)
