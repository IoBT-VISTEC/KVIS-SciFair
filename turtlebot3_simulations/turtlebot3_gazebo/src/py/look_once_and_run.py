from turtlebot_drive import Turtlebot3_drive
import rospy

class drive(Turtlebot3_drive):

    def logic(self):
        if self.initial:
            self.initial = False
            return "walk"

        # 1. Declare variable to know that TTB got a target direction. 0 = not get direction, 1 = get direction
        get_direction = self.mem[0] 

        # when return "run" it will exit function
        if get_direction == 1:
            return "run"

        # 2. Seek for the shortest and turn to its direction
        center_dist = self.top_center_sensor - self.bottom_center_sensor
        left_dist = self.top_left_sensor - self.bottom_left_sensor
        right_dist = self.top_right_sensor - self.top_right_sensor

        if center_dist < left_dist:
            if center_dist < right_dist:
                min_dist = center_dist
            else:
                min_dist = right_dist
        else:
            if left_dist < right_dist:
                min_dist = left_dist 
            else:
                min_dist = right_dist
        
        # When it hit the wall, stop and turn right
        if min_dist == 0:
            if self.current_vel != 0:
                return "stop"
            else:
                return "turn right"

        # 3. turn to direction and set get_direction
        get_direction = 1
        if min_dist == center_dist:
            return "run"
        elif min_dist == right_dist:
            return "turn right"
        elif min_dist == left_dist:
            return "turn left"


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