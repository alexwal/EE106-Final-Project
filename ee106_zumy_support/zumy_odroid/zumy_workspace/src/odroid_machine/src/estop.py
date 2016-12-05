#!/usr/bin/python

from Tkinter import Tk, Label, Button

import rospy
import time
from std_msgs.msg import String,Bool,Float32
from zumy_ros.msg import ZumyStatus

check_time = time.time() #timer used so that the watchdog doesn't get published at an insane rate.

class GUI:
    def __init__(self, master):

        self.master = master

        rospy.init_node('base_GUI')
        self.robot = rospy.get_param('~mname') #private value, but I need to know which robot i'm controlling.
        #self.robot = 'zumy7a'
        
        if rospy.has_param("~timeout"):
            self.timeout = rospy.get_param('~timeout') #private value
        else:
            self.timeout = 2 #Length of watchdog timer in seconds, defaults to 2 sec.
        
        master.title(self.robot)

        self.label = Label(master, text="Ctrl-c in the launch window to stop").grid(row=1,column=0)

        self.robot_state_label = Label(master,text='Robot is enabled')
        self.robot_state_label.grid(row=1,column=1)

        self.voltage = 0
        self.voltage_label = Label(master,text = "VBatt = " + str(self.voltage))
        self.voltage_label.grid(row=2,column=1)

        self.loop_freq = 0
        self.loop_freq_label = Label(master,text = "Zumy Ros Freq: " + str(self.loop_freq) + " Hz")
        self.loop_freq_label.grid(row=3,column = 1)

        self.last_heard_label = Label(master,text = "Last Heard = Never")
        self.last_heard_label.grid(row=4,column = 1)


        self.enable_button = Button(master, text="Disable", command=self.change_enable_state)
        self.enable_button. grid(row=2,column=0)
        self.enabled = True

        self.close_button = Button(master, text="ESTOP", command=self.estop,bg='red').grid(row=3,column=0)

        self.battery_unsafe = False

        #ROS stuff


        self.heartbeat_pub = rospy.Publisher('/base_computer',String,queue_size=1) #/base_computer topic, the global watchdog.  May want to investigate what happens when there moer than one computer and more than one zumy
        self.zumy_heartbeat = rospy.Subscriber('/' + self.robot + '/Status',ZumyStatus,self.callback,queue_size = 1)
        self.zumy_enable = rospy.Publisher('/' + self.robot + '/enable',Bool,queue_size = 1) #The GUI actuates the publish topic.
        self.zumy_voltage = rospy.Subscriber('/'+self.robot+'/Batt',Float32,self.voltage_callback,queue_size = 1)

        self.last_heard = time.time()



    def change_enable_state(self):
        if self.battery_unsafe:
            self.enabled = False
            self.enable_button["text"] = "  "
            self.robot_state_label.config(text = "Robot is disabled \n CHARGE YOUR BATTERY")
        elif self.enabled:
            self.enabled = False
            #Robot is disabled, the button is to enable
            self.enable_button["text"] = "Enable"
            self.robot_state_label.config(text = "Robot is disabled")
            self.zumy_enable.publish(Bool(self.enabled)) #Change enabled state
        else:
            self.enabled = True
            #Robot is enabled, the button will disable
            self.enable_button["text"] = "Disable"
            self.robot_state_label.config(text = "Robot is enabled")
            self.zumy_enable.publish(Bool(self.enabled)) #Change enabled state


    def estop(self):
        self.zumy_enable.publish(Bool(False)) #Bool is the ROS bool.  Disable the robot, please.
        self.enable_button["text"] = "Enable"
        self.robot_state_label.config(text = "Robot is disabled")
        self.enabled = False
        pass

    def callback(self,msg):
        self.last_heard = time.time()
        if msg.battery_unsafe:
        	#my battery is unsafe... irreversably latch battery_unsafe to True, and the disable the robot.
            self.battery_unsafe = True
            self.change_enable_state()
        self.loop_freq = msg.loop_freq
        self.loop_freq_label["text"] = "Zumy Ros Freq: " + (" %.2f" % self.loop_freq)  + " Hz"



    def last_heard_text(self):
        dt_ms = int(1000*(time.time() - self.last_heard))
        if dt_ms < 1000:
            self.last_heard_label["text"] = "Last Heard (ms):   " + '{0:04d}'.format(dt_ms)
        else:
            self.last_heard_label["text"] = "Last Heard (ms):  > 1000"



    def voltage_callback(self,msg):
        self.voltage = float(msg.data)
        self.voltage_label.config(text = "VBatt = " + ("%.2f" % round(self.voltage,2)) + " V" )




def check():
    #force Tkinter to do something every 250ms... which is useful so it'll notice ctr-c in a reasonable timeframe
    
    if not rospy.is_shutdown():  #so if rospy is shutdown (say, ctrl-c), the window gets closed too.
        my_gui.last_heard_text()
        
        global check_time

        if  (check_time + .25) < time.time():
            my_gui.heartbeat_pub.publish(String("Foo")) #publish the computer's watchdog
            check_time = time.time()


        root.after(50, check) # 250 stands for 250 ms.
        
        


        
        #
    else:
        root.destroy()


if __name__ == '__main__':
    



    root = Tk()
    my_gui = GUI(root)
    root.after(250, check)
    root.mainloop()

#root = Tk()
#my_gui = MyFirstGUI(root)
#root.mainloop()