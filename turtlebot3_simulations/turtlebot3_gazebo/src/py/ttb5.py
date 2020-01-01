from turtlebot_drive import Turtlebot3_drive
import rospy

class drive(Turtlebot3_drive):

    def logic(self):
        if self.initial:
            self.initial = False
            return "walk"

        if self.top_center_sensor - self.bottom_center_sensor > 0.1:
            print('seek')
            if self.bottom_center_sensor > self.bottom_left_sensor:
                return "turn left"
            elif self.bottom_center_sensor > self.bottom_right_sensor:
                return "turn right"
            else:
                return "run"
            
        else:
            print('avoid')
            if self.bottom_right_sensor > self.bottom_left_sensor:
                return "turn right"
            else:
                return "turn left"

if __name__ == '__main__':
    rospy.init_node('drive', anonymous=True)
    rate = rospy.Rate(10)
    try:
        ttb = drive("purple")
        while not rospy.is_shutdown():
            ttb.controlLoop()
            rate.sleep()
    except rospy.ROSInterruptException:
        ttb.updatecommandVelocity(0.0, 0.0)