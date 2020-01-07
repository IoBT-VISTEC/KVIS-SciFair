from turtlebot_drive import Turtlebot3_drive
import rospy

class drive(Turtlebot3_drive):

    def logic(self):
        if self.initial:
            self.initial = False
            return "walk"
        '''
        This exmaple TTB will run til hit a wall or soda can
        When it hit the wall, stop and turn around. Then run again
        '''

        center_dist = self.top_center_sensor - self.bottom_center_sensor
        turn_right_count = self.mem[0] 

        if center_dist < 0.1:
            if self.current_vel != 0:
                return "stop"
            else:
                if turn_right_count > 8:
                    self.mem[0] = 0  # important! reset count to zero
                    return "run"
                else:
                    turn_right_count += 1
                    self.mem[0] = turn_right_count  
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