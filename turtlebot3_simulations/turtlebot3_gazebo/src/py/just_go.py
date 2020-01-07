from turtlebot_drive import Turtlebot3_drive
import rospy

class drive(Turtlebot3_drive):

    def logic(self):
        if self.initial:
            self.initial = False
            return "walk"

        # Find target. only in front of TTB
        center_dist = self.top_center_sensor - self.bottom_center_sensor

        # When it hit the wall, stop and turn right until find new target
        if center_dist == 0:
            if self.current_vel != 0:
                return "stop"
            else:
                return "turn right"
        else:
            return "run"


if __name__ == '__main__':
    rospy.init_node('drive', anonymous=True)
    rate = rospy.Rate(10)
    try:
        ttb = drive("yellow")
        while not rospy.is_shutdown():
            ttb.controlLoop()
            rate.sleep()
    except rospy.ROSInterruptException:
        ttb.updatecommandVelocity(0.0, 0.0)