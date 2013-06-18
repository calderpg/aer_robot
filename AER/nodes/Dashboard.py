#!/usr/bin/python

from DriveControllers import *
import Tkinter
from Tkinter import *
from Tkconstants import *

import roslib; roslib.load_manifest('AER')
import rospy

from geometry_msgs.msg import Twist
from AER.msg import Sensor
from AER.msg import Bumper
from AER.msg import Power
from AER.msg import Motor


class Visualizer(Tkinter.Frame):
    def __init__(self, real):
        rospy.init_node('Visualizer')
        Tkinter.Frame.__init__(self, root)
        self.controller = MecanumDrive()
        #Set up the callbacks
        rospy.Subscriber("cmd_vel", Twist, self.TwistCallback)
        rospy.Subscriber("aer_bumpers", Bumper, self.BumperCallback)
        #rospy.Subscriber("aer_sensors", Sensor, self.SensorCallback)
        rospy.Subscriber("aer_power", Power, self.PowerCallback)
        rospy.Subscriber("aer_motors", Motor, self.MotorCallback)
        #Starting values
        #Commanded motor values
        self.LFcmd = 0.0
        self.RFcmd = 0.0
        self.LRcmd = 0.0
        self.RRcmd = 0.0
        #Real motor values (from encoders)
        self.LFreal = 0.0
        self.RFreal = 0.0
        self.LRreal = 0.0
        self.RRreal = 0.0
        #Power bus voltage (default is 12V)
        self.power = 15.0
        #Bumper values
        self.RFF = 0
        self.RFS = 0
        self.RRS = 0
        self.RRB = 0
        self.LRB = 0
        self.LRS = 0
        self.LFS = 0
        self.LFF = 0
        #Draw the general window labels
        self.headLabel = Label(self, justify=CENTER, font=("Helvetica", 18))
        self.headLabel["text"] = "Vehicle Status"
        self.headLabel.grid(row=0, columnspan=8)
        self.line1 = Label(self, text="------------------------------------------------------------", font=("Helvetica", 14))
        self.line1.grid(row=1, columnspan=8)
        self.line2 = Label(self, text="------------------------------------------------------------", font=("Helvetica", 14))
        self.line2.grid(row=5, columnspan=8)
        self.line3 = Label(self, text="------------------------------------------------------------", font=("Helvetica", 14))
        self.line3.grid(row=11, columnspan=8)
        self.line4 = Label(self, text="------------------------------------------------------------", font=("Helvetica", 14))
        self.line4.grid(row=15, columnspan=8)
        self.line5 = Label(self, text="------------------------------------------------------------", font=("Helvetica", 14))
        self.line5.grid(row=17, columnspan=8)
        self.line5 = Label(self, text="------------------------------------------------------------", font=("Helvetica", 14))
        self.line5.grid(row=19, columnspan=8)
        #Draw motor values
        #Draw labels
        self.LFlabel = Label(self, text="Left Front", font=("Helvetica", 12))
        self.LFlabel.grid(row=2, column=0, columnspan=2)
        self.RFlabel = Label(self, text="Right Front", font=("Helvetica", 12))
        self.RFlabel.grid(row=2, column=6, columnspan=2)
        self.LRlabel = Label(self, text="Left Rear", font=("Helvetica", 12))
        self.LRlabel.grid(row=12, column=0, columnspan=2)
        self.RRlabel = Label(self, text="Right Rear", font=("Helvetica", 12))
        self.RRlabel.grid(row=12, column=6, columnspan=2)
        self.MC1label = Label(self, text="CMD", font=("Helvetica", 10))
        self.MC1label.grid(row=3, column=0)
        self.MC2label = Label(self, text="CMD", font=("Helvetica", 10))
        self.MC2label.grid(row=3, column=6)
        self.MC3label = Label(self, text="CMD", font=("Helvetica", 10))
        self.MC3label.grid(row=13, column=0)
        self.MC4label = Label(self, text="CMD", font=("Helvetica", 10))
        self.MC4label.grid(row=13, column=6)
        self.MR1label = Label(self, text="Real", font=("Helvetica", 10))
        self.MR1label.grid(row=3, column=1)
        self.MR2label = Label(self, text="Real", font=("Helvetica", 10))
        self.MR2label.grid(row=3, column=7)
        self.MR3label = Label(self, text="Real", font=("Helvetica", 10))
        self.MR3label.grid(row=13, column=1)
        self.MR4label = Label(self, text="Real", font=("Helvetica", 10))
        self.MR4label.grid(row=13, column=7)
        self.Bumperlabel = Label(self, text="Bumpers", font=("Helvetica", 12))
        self.Bumperlabel.grid(row=6, column=2, columnspan=4)
        self.Keylabel = Label(self, text="Advanced Educational Robot ROS Driver rev. 1", font=("Helvetica", 12))
        self.Keylabel.grid(row=18, column=0, columnspan=8)
        #Get color values
        LFCcolor = self.colorize(self.LFcmd)
        RFCcolor = self.colorize(self.RFcmd)
        LRCcolor = self.colorize(self.LRcmd)
        RRCcolor = self.colorize(self.RRcmd)
        LFEcolor = self.colorize(self.LFreal)
        RFEcolor = self.colorize(self.RFreal)
        LREcolor = self.colorize(self.LRreal)
        RREcolor = self.colorize(self.RRreal)
        #Get command values
        LFCv = self.valuize(self.LFcmd)
        RFCv = self.valuize(self.RFcmd)
        LRCv = self.valuize(self.LRcmd)
        RRCv = self.valuize(self.RRcmd)
        LFEv = self.valuize(self.LFreal)
        RFEv = self.valuize(self.RFreal)
        LREv = self.valuize(self.LRreal)
        RREv = self.valuize(self.RRreal)
        #Draw values
        self.LFCvalue = Label(self, text=LFCv, bg=LFCcolor, fg="white", font=("Courier", 18))
        self.LFCvalue.grid(row=4, column=0)
        self.RFCvalue = Label(self, text=RFCv, bg=RFCcolor, fg="white", font=("Courier", 18))
        self.RFCvalue.grid(row=4, column=6)
        self.LRCvalue = Label(self, text=LRCv, bg=LRCcolor, fg="white", font=("Courier", 18))
        self.LRCvalue.grid(row=14, column=0)
        self.RRCvalue = Label(self, text=RRCv, bg=RRCcolor, fg="white", font=("Courier", 18))
        self.RRCvalue.grid(row=14, column=6)
        self.LFEvalue = Label(self, text=LFEv, bg=LFEcolor, fg="white", font=("Courier", 18))
        self.LFEvalue.grid(row=4, column=1)
        self.RFEvalue = Label(self, text=RFEv, bg=RFEcolor, fg="white", font=("Courier", 18))
        self.RFEvalue.grid(row=4, column=7)
        self.LREvalue = Label(self, text=LREv, bg=LREcolor, fg="white", font=("Courier", 18))
        self.LREvalue.grid(row=14, column=1)
        self.RREvalue = Label(self, text=RREv, bg=RREcolor, fg="white", font=("Courier", 18))
        self.RREvalue.grid(row=14, column=7)
        #Draw Bumper values
        #Get colors
        LFFcolor = self.bumpercolorize(self.LFF)
        LFScolor = self.bumpercolorize(self.LFS)
        LRScolor = self.bumpercolorize(self.LRS)
        LRBcolor = self.bumpercolorize(self.LRB)
        RFFcolor = self.bumpercolorize(self.RFF)
        RFScolor = self.bumpercolorize(self.RFS)
        RRScolor = self.bumpercolorize(self.RRS)
        RRBcolor = self.bumpercolorize(self.RRB)
        #Get values
        LFFval = self.bumperize(self.LFF)
        LFSval = self.bumperize(self.LFS)
        LRSval = self.bumperize(self.LRS)
        LRBval = self.bumperize(self.LRB)
        RFFval = self.bumperize(self.RFF)
        RFSval = self.bumperize(self.RFS)
        RRSval = self.bumperize(self.RRS)
        RRBval = self.bumperize(self.RRB)
        #Draw values
        self.LFFlabel = Label(self, text=LFFval, bg=LFFcolor, fg="white", font=("Courier", 18))
        self.LFFlabel.grid(row=7, column=3)
        self.LFSlabel = Label(self, text=LFSval, bg=LFScolor, fg="white", font=("Courier", 18))
        self.LFSlabel.grid(row=8, column=2)
        self.LRSlabel = Label(self, text=LRSval, bg=LRScolor, fg="white", font=("Courier", 18))
        self.LRSlabel.grid(row=9, column=2)
        self.LRBlabel = Label(self, text=LRBval, bg=LRBcolor, fg="white", font=("Courier", 18))
        self.LRBlabel.grid(row=10, column=3)
        self.RFFlabel = Label(self, text=RFFval, bg=RFFcolor, fg="white", font=("Courier", 18))
        self.RFFlabel.grid(row=7, column=4)
        self.RFSlabel = Label(self, text=RFSval, bg=RFScolor, fg="white", font=("Courier", 18))
        self.RFSlabel.grid(row=8, column=5)
        self.RRSlabel = Label(self, text=RRSval, bg=RRScolor, fg="white", font=("Courier", 18))
        self.RRSlabel.grid(row=9, column=5)
        self.RRBlabel = Label(self, text=RRBval, bg=RRBcolor, fg="white", font=("Courier", 18))
        self.RRBlabel.grid(row=10, column=4)
        #Draw power status
        #Get value
        PowerVal = self.powerize(self.power)
        #Get color
        PowerColor = self.powercolorize(self.power)
        #Draw value
        self.Powerlabel = Label(self, text="Main bus voltage:", font=("Helvetica", 12))
        self.Powerlabel.grid(row=16, column=0, columnspan=4)
        self.PowerValuelabel = Label(self, text=PowerVal, bg=PowerColor, fg="white", font=("Courier", 18))
        self.PowerValuelabel.grid(row=16, column=4, columnspan=4)

    def BumperCallback(self, data):
        self.RFF = data.bumper1
        self.RFS = data.bumper2
        self.RRS = data.bumper3
        self.RRB = data.bumper4
        self.LRB = data.bumper5
        self.LRS = data.bumper6
        self.LFS = data.bumper7
        self.LFF = data.bumper8
        #Draw Bumper values
        #Get colors
        LFFcolor = self.bumpercolorize(self.LFF)
        LFScolor = self.bumpercolorize(self.LFS)
        LRScolor = self.bumpercolorize(self.LRS)
        LRBcolor = self.bumpercolorize(self.LRB)
        RFFcolor = self.bumpercolorize(self.RFF)
        RFScolor = self.bumpercolorize(self.RFS)
        RRScolor = self.bumpercolorize(self.RRS)
        RRBcolor = self.bumpercolorize(self.RRB)
        #Get values
        LFFval = self.bumperize(self.LFF)
        LFSval = self.bumperize(self.LFS)
        LRSval = self.bumperize(self.LRS)
        LRBval = self.bumperize(self.LRB)
        RFFval = self.bumperize(self.RFF)
        RFSval = self.bumperize(self.RFS)
        RRSval = self.bumperize(self.RRS)
        RRBval = self.bumperize(self.RRB)
        #Draw values
        self.LFFlabel = Label(self, text=LFFval, bg=LFFcolor, fg="white", font=("Courier", 18))
        self.LFFlabel.grid(row=7, column=3)
        self.LFSlabel = Label(self, text=LFSval, bg=LFScolor, fg="white", font=("Courier", 18))
        self.LFSlabel.grid(row=8, column=2)
        self.LRSlabel = Label(self, text=LRSval, bg=LRScolor, fg="white", font=("Courier", 18))
        self.LRSlabel.grid(row=9, column=2)
        self.LRBlabel = Label(self, text=LRBval, bg=LRBcolor, fg="white", font=("Courier", 18))
        self.LRBlabel.grid(row=10, column=3)
        self.RFFlabel = Label(self, text=RFFval, bg=RFFcolor, fg="white", font=("Courier", 18))
        self.RFFlabel.grid(row=7, column=4)
        self.RFSlabel = Label(self, text=RFSval, bg=RFScolor, fg="white", font=("Courier", 18))
        self.RFSlabel.grid(row=8, column=5)
        self.RRSlabel = Label(self, text=RRSval, bg=RRScolor, fg="white", font=("Courier", 18))
        self.RRSlabel.grid(row=9, column=5)
        self.RRBlabel = Label(self, text=RRBval, bg=RRBcolor, fg="white", font=("Courier", 18))
        self.RRBlabel.grid(row=10, column=4)

    def PowerCallback(self, data):
        self.power = data.input
        #Get value
        PowerVal = self.powerize(self.power)
        #Get color
        PowerColor = self.powercolorize(self.power)
        #Draw value
        self.PowerValuelabel = Label(self, text=PowerVal, bg=PowerColor, fg="white", font=("Courier", 18))
        self.PowerValuelabel.grid(row=16, column=4, columnspan=4)

    def powerize(self, value):
        """Convert power value to a string"""
        return "+" + str(value)

    def powercolorize(self, value):
        red = 0
        blue = 0
        green = 0        
        if value != 0.0:
            green = 200
        if value > 14.0:
            green = 200
            blue = 200
        tk_rgb = "#%02x%02x%02x" % (red, green, blue)
        return tk_rgb

    def bumperize(self, value):
        """Convert bumper value to a string"""
        if value == 0:
            return "-"
        else:
            return "*"

    def bumpercolorize(self, value):
        red = 0
        blue = 0
        green = 0        
        if value != 0:
            red = 255
        tk_rgb = "#%02x%02x%02x" % (red, green, blue)
        return tk_rgb
        

    def valuize(self, value):
        """Convert speed value into string value"""
        raw_value = int(127 * value)
        if (raw_value <= 127 and raw_value >= 100):
            return "+" + str(raw_value)
        elif (raw_value <= 99 and raw_value >= 10):
            return "+0" + str(raw_value)
        elif (raw_value <= 9 and raw_value > 0):
            return "+00" + str(raw_value)
        elif (raw_value == 0):
            return "0000"
        elif (raw_value < 0 and raw_value >= -9):
            return "-00" + str(abs(raw_value))
        elif (raw_value < 9 and raw_value >= -99):
            return "-0" + str(abs(raw_value))
        elif (raw_value < -99 and raw_value >= -127):
            return "-" + str(abs(raw_value))
            

    def colorize(self, value):
        """Convert speed value into color"""
        blue = 0
        green = 0
        red = 0
        if (value == 0.0 or value == -0.0):
            green = 0
            red = 0
        elif (value > 0.0):
            red = 0
            green = int(255 * value)
        elif (value < -0.0):
            green = 0
            red = int(255 * abs(value))
        else:
            green = 0
            red = 0
            blue = 255
        tk_rgb = "#%02x%02x%02x" % (red, green, blue)
        return tk_rgb

    def MotorCallback(self, data):
        self.LFreal = data.front.M0
        self.RFreal = data.front.M1
        self.LRreal = data.rear.M0
        self.RRreal = data.rear.M1
        #Get color values
        LFEcolor = self.colorize(self.LFreal)
        RFEcolor = self.colorize(self.RFreal)
        LREcolor = self.colorize(self.LRreal)
        RREcolor = self.colorize(self.RRreal)
        #Get encoder values
        LFEv = self.valuize(self.LFreal)
        RFEv = self.valuize(self.RFreal)
        LREv = self.valuize(self.LRreal)
        RREv = self.valuize(self.RRreal)
        #Draw values
        self.LFEvalue = Label(self, text=LFEv, bg=LFEcolor, fg="white", font=("Courier", 18))
        self.LFEvalue.grid(row=4, column=1)
        self.RFEvalue = Label(self, text=RFEv, bg=RFEcolor, fg="white", font=("Courier", 18))
        self.RFEvalue.grid(row=4, column=7)
        self.LREvalue = Label(self, text=LREv, bg=LREcolor, fg="white", font=("Courier", 18))
        self.LREvalue.grid(row=14, column=1)
        self.RREvalue = Label(self, text=RREv, bg=RREcolor, fg="white", font=("Courier", 18))
        self.RREvalue.grid(row=14, column=7)

    def TwistCallback(self, data):
        X = data.linear.x
        Y = data.linear.y
        Z = data.angular.z
        #print "X : " + str(X) + " Y : " + str(Y) + " Z : " + str(Z)
        commands = self.controller.Compute(X, Y, Z)
        self.LFcmd = commands[0]
        self.RFcmd = commands[1]
        self.LRcmd = commands[2]
        self.RRcmd = commands[3]
        #Get color values
        LFCcolor = self.colorize(self.LFcmd)
        RFCcolor = self.colorize(self.RFcmd)
        LRCcolor = self.colorize(self.LRcmd)
        RRCcolor = self.colorize(self.RRcmd)
        #Get command values
        LFCv = self.valuize(self.LFcmd)
        RFCv = self.valuize(self.RFcmd)
        LRCv = self.valuize(self.LRcmd)
        RRCv = self.valuize(self.RRcmd)
        #Draw values
        self.LFCvalue = Label(self, text=LFCv, bg=LFCcolor, fg="white", font=("Courier", 18))
        self.LFCvalue.grid(row=4, column=0)
        self.RFCvalue = Label(self, text=RFCv, bg=RFCcolor, fg="white", font=("Courier", 18))
        self.RFCvalue.grid(row=4, column=6)
        self.LRCvalue = Label(self, text=LRCv, bg=LRCcolor, fg="white", font=("Courier", 18))
        self.LRCvalue.grid(row=14, column=0)
        self.RRCvalue = Label(self, text=RRCv, bg=RRCcolor, fg="white", font=("Courier", 18))
        self.RRCvalue.grid(row=14, column=6)


if __name__ == "__main__":
    root = Tkinter.Tk()
    root.title("AER Dashboard")
    Visualizer(root).pack()
    root.mainloop()
