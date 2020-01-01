import rospy
from std_msgs.msg import String
from Tkinter import *

root = Tk(className="Signal Trigger")
var1 = StringVar()
var2 = StringVar()
var1.set("...")
var2.set("...")
var3 = StringVar()
var3.set("Direction")

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
    greenbutton["state"] = "disabled"
    eeg_pub.publish("action")


root.minsize(300,50)
upperframe = Frame(root)
upperframe.pack(side=TOP)
frame = Frame(root) 
frame.pack() 
lowerframe = Frame(root)
lowerframe.pack(side=BOTTOM)
greenbutton = Button(frame, text = 'Start', fg ='green', command=callback) 
greenbutton.configure(font="-family {Cascadia Code} -size 36")
greenbutton.pack( side = LEFT) 
redbutton = Button(frame, text = 'Stop', fg='red')# , command=hideReady
redbutton.configure(font="-family {Cascadia Code} -size 36")
redbutton.pack( side = LEFT ) 
label_left = Label(lowerframe, textvariable=var1, relief=SUNKEN )
label_left.configure(font="-family {Cascadia Code} -size 36")
label_left.pack(side=LEFT)
label_right = Label(lowerframe, textvariable=var2, relief=SUNKEN )
label_right.configure(font="-family {Cascadia Code} -size 36")
label_right.pack(side=RIGHT)
label_top = Label(upperframe, textvariable=var3, relief=SUNKEN )
label_top.configure(font="-family {Cascadia Code} -size 36")
label_top.pack(side=TOP)
root.mainloop() 