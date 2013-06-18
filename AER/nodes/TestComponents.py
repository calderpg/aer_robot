#
# Testing components for use in the testing of code
# for the Advanced Educational Robot, including a
# simulator for the host-microcontroller link.
#
#
import time
from PlatformComponents import *

class EMUplatform():

    def __init__(self, port, rate, name="Emulator"):
        self.name = name
        
    def __str__(self):
        return self.name

    def Reset(self):
        time.sleep(1)
        print "Platform initialized successfully.\nVehicle now ready."

    def GetState(self):
        LF_speed = 1.0
        RF_speed = 1.0
        LR_speed = 1.0
        RR_speed = 1.0
        new_state = MotorState(LF_speed, RF_speed, LR_speed, RR_speed)
        return new_state

    def SetModeAuto(self):
        print "Set to auto mode"

    def SetModeManual(self):
        print "Set to manual mode"

    def Brake(self):
        print "Braking"

    def Stop(self):
        print "Stopping"

    def Go(self, LeftFront, RightFront, RightRear, LeftRear):
        self.boundsCheck(LeftFront, RightFront, LeftRear, RightRear)
        commandString = 'C' + str(LeftFront) + '|' + str(RightFront) + '\n' + str(LeftRear) + '|' + str(RightRear) + '\n'
        print commandString

    def boundsCheck(self, LF, RF, LR, RR):
        if (abs(LF) > 1.0 or abs(RF) > 1.0 or abs(LR) > 1.0 or abs(RR) > 1.0):
            print "***Exceded motor command bounds!***"
            raise BaseException
