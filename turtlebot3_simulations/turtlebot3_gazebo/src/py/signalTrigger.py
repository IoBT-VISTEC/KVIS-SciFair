import rospy
import math
from std_msgs.msg import String
from Tkinter import *
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

root = Tk(className="Signal Trigger")
var1 = StringVar()
var2 = StringVar()
var1.set("...")
var2.set("...")
var3 = StringVar()
var3.set("Direction")
mapNo = StringVar()
mapNo.set(1)
mapLabel = StringVar()
mapLabel.set("MAP:")
turn = StringVar()
turn.set(0)
turnLabel = StringVar()
turnLabel.set("Turn:")

pos = [
        [1.5,-1.5],
        [-9.5, 6.5],
        [9.5, -9.5],
        [-8.5, -6.5],
        [-9.5,1.5],
        [9.5, 9.5],  
    ]

def displayReady(msg):
    #var.set("Start the movement...")
    #var.set(msg.data)
    if msg.data == "action":
        return 0
    if msg.data == "1":
        var1.set("True")
        var2.set("False")
        var3.set("Turn Left")
    elif msg.data == "2":
        var1.set("False")
        var2.set("True")
        var3.set("Turn Right")
    elif msg.data == "3":
        var1.set("False")
        var2.set("False")
        var3.set("Go Forward")
    else:
        var1.set("True")
        var2.set("True")
        var3.set("Go Backward")
    greenbutton["state"] = "normal"

#def hideReady():
    #var.set("")

rospy.init_node('eeg_signal', anonymous=True)
eeg_pub = rospy.Publisher("/chatter", String, queue_size=10)
eeg_sub = rospy.Subscriber("/chatter", String, displayReady)

def callback():
    #var1.set("Sending Request...")
    turn.set(int(turn.get()) + 1)
    greenbutton["state"] = "disabled"
    eeg_pub.publish("action")

def nextMap():
    maze = int(mapNo.get())
    if maze == 6:
        maze = 1
    else:
        maze = maze + 1
    mapNo.set(maze)

    a = ModelState()
    a.model_name = "turtlebot3"
    a.pose.position.x = pos[maze-1][0]
    a.pose.position.y = pos[maze-1][1]
    a.pose.position.z = 0.15
    
    a.pose.orientation.x = 0
    a.pose.orientation.y = 0
    a.pose.orientation.z = math.sqrt(2)/2
    a.pose.orientation.w = math.sqrt(2)/2

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( a )
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


root.minsize(300,50)
upframe = Frame(root)
upframe.pack(side=TOP)
upperframe = Frame(root)
upperframe.pack(side=TOP)
frame = Frame(root) 
frame.pack() 
lowestframe = Frame(root)
lowestframe.pack(side=BOTTOM)
lowerframe = Frame(root)
lowerframe.pack(side=BOTTOM)
greenbutton = Button(frame, text = 'Start', fg ='green', command=callback) 
greenbutton.configure(font="-family {Cascadia Code} -size 36")
greenbutton.pack( side = LEFT) 
redbutton = Button(lowestframe, text = 'Next', fg='red', command=nextMap) 
redbutton.configure(font="-family {Cascadia Code} -size 36")
redbutton.pack( side = BOTTOM ) 
label_left = Label(lowerframe, textvariable=var1, relief=SUNKEN )
label_left.configure(font="-family {Cascadia Code} -size 36")
label_left.pack(side=LEFT)
label_right = Label(lowerframe, textvariable=var2, relief=SUNKEN )
label_right.configure(font="-family {Cascadia Code} -size 36")
label_right.pack(side=RIGHT)
label_top = Label(upperframe, textvariable=var3, relief=SUNKEN )
label_top.configure(font="-family {Cascadia Code} -size 36")
label_top.pack(side=TOP)
label_map = Label(lowestframe, textvariable=mapNo, relief=RIDGE )
label_map.configure(font="-family {Cascadia Code} -size 36")
label_map.pack(side=RIGHT)
label_map_count = Label(lowestframe, textvariable=mapLabel, relief=GROOVE )
label_map_count.configure(font="-family {Cascadia Code} -size 36")
label_map_count.pack(side=LEFT)

label_turn = Label(upframe, textvariable=turnLabel, relief=RIDGE )
label_turn.configure(font="-family {Cascadia Code} -size 36")
label_turn.pack(side=LEFT)
counter = Label(upframe, textvariable=turn, relief=GROOVE )
counter.configure(font="-family {Cascadia Code} -size 36")
counter.pack(side=LEFT)
root.mainloop() 
