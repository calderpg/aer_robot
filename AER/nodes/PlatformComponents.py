#
#   Host class for communication and control of
#   an "advanced educational robot" via the serial
#   connection to the onboard microcontroller.
#
#   The reference implementation of the AER uses a
#   Mbed (ARM Cortex-M3) microcontroller for onboard
#   control; you may replace it with another platform
#   so long as the interface defined below is maintained.
#
#   Authors :   Calder Phillips-Grafflin (Union College)
#               Erik Skorina (Union College)
#
#   For license information, see project license in the
#   root directory of this software.
#

import serial
import time
import math

class SensorState():

    def __init__(self, X_accel, Y_accel, Z_accel, Z_gyro):
        self.X_A = X_accel
        self.Y_A = Y_accel
        self.Z_A = Z_accel
        self.Z_R = Z_gyro

    def __str__(self):
        string_representation = "Accelerometer :\nX : " + str(self.X_A) + "\nY : " + str(self.Y_A) + "\nZ : " + str(self.Z_A) + "\nGyroscope : \nZ : " + str(self.Z_R)
        return string_representation

class BumperState():

    def __init__(self, b1, b2, b3, b4, b5, b6, b7, b8):
        self.RFF = b1
        self.RFS = b2
        self.RRS = b3
        self.RRB = b4
        self.LRB = b5
        self.LRS = b6
        self.LFS = b7
        self.LFF = b8

    def __str__(self):
        string_representation = str(self.RFF) + "," + str(self.RFS) + "," + str(self.RRS) + "," + str(self.RRB) + "," + str(self.LRB) + "," +str(self.LRS) + "," + str(self.LFS) + "," + str(self.LFF)
        return string_representation

class MotorState():

    """
    Stores a set of 4 motor speeds (a 'state') from the microcontroller
    """
    
    def __init__(self, LF_speed, RF_speed, LR_speed, RR_speed, error1, error2):
        self.LF = LF_speed
        self.RF = RF_speed
        self.LR = LR_speed
        self.RR = RR_speed
        self.MC1_error = error1 #Stores error codes from the front motor controller
        self.MC2_error = error2 #Stores error codes from the rear motor controller

    def __str__(self):
        string_representation =  "Motor Speeds :\nLeft Front (LF) : " + str(self.LF) + "\nRight Front (RF) : " + str(self.RF) + "\nLeft Rear (LR) : " + str(self.LR) + "\nRight Rear (RR) : " + str(self.RR) + "\nQik Errors: " + str(self.MC1_error) + ", " + str(self.MC2_error)
        return string_representation

class AERsensor():

    def __init__(self, port, rate, name="Sensors"):
        self.micro = serial.Serial(port, rate, 8, 'N', 1, timeout=0.1)
        self.name = name
        print "Sensor platform ready"

    def __str__(self):
        return self.name

    def GetState(self):
        try:
            self.micro.write('G')
            raw_data = self.micro.readline()
            print "Got: " + raw_data
            analog_chunks = raw_data.split("$")
            raw_state = analog_chunks[0]
            state_chunks = raw_state.split("|")
            X_accel = float(state_chunks[0])
            Y_accel = float(state_chunks[1])
            Z_accel = float(state_chunks[3])
            Z_gyro = float(state_chunks[2])
            new_state1 = SensorState(X_accel, Y_accel, Z_accel, Z_gyro)
            raw_state2 = analog_chunks[1]
            bumper_chunks = raw_state2.split("|")
            b1 = int(bumper_chunks[0]) #replace with 1
            b2 = int(bumper_chunks[1]) #replace with 2
            b3 = int(bumper_chunks[2]) #replace with 0/3
            b4 = int(bumper_chunks[3]) #replace with 3/0
            b5 = int(bumper_chunks[4]) #replace with 5
            b6 = int(bumper_chunks[5]) #replace with 6
            b7 = int(bumper_chunks[6]) #replace with 7
            b8 = int(bumper_chunks[7]) #replace with 4
            new_state2 = BumperState(b1, b2, b3, b4, b5, b6, b7, b8)
            print "Produced: " + str(new_state2)
            return [new_state1, new_state2]
        except:
            new_state1 = SensorState(-1.0, -1.0, -1.0, -1.0)
            new_state2 = BumperState(-1, -1, -1, -1, -1, -1, -1, -1)
            return [new_state1, new_state2]

class AERplatform():

    def __init__(self, port, rate, name="Platform"):
        self.micro = serial.Serial(port, rate, 8, 'N', 1, timeout=0.1)
        self.name = name
        self.micro.write('R')
        reply = self.micro.readline()
        while "Ready!" not in reply:
            reply = self.micro.readline()
        print "Platform initialized successfully.\nVehicle now ready."

    def __str__(self):
        return self.name

    def Reset(self):
        try:
            self.micro.write('R')
            reply = self.micro.readline()
            while "Ready!" not in reply:
                reply = self.micro.readline()
            print "Platform initialized successfully.\nVehicle now ready."
        except:
            print "UNABLE TO WRITE TO THE SERIAL PORT...!"

    def GetState(self):
        try:
            self.micro.write('G')
            raw_state = self.micro.readline()
            state_chunks = raw_state.split("|")
            LF_speed = float(state_chunks[0])
            RF_speed = float(state_chunks[1])
            LR_speed = float(state_chunks[3])
            RR_speed = float(state_chunks[2])
            MC1_error = state_chunks[4]
            MC2_error = state_chunks[5]
            new_state = MotorState(LF_speed, RF_speed, LR_speed, RR_speed, MC1_error, MC2_error)
            return new_state
        except:
            new_state = MotorState(0.0, 0.0, 0.0, 0.0, '!', '!')
            return new_state

    def SetModeAuto(self):
        try:
            self.micro.write('A')
        except:
            print "UNABLE TO WRITE TO THE SERIAL PORT...!"

    def SetModeManual(self):
        try:
            self.micro.write('M')
        except:
            print "UNABLE TO WRITE TO THE SERIAL PORT...!"

    def Brake(self):
        try:
            self.micro.write('B')
        except:
            print "UNABLE TO WRITE TO THE SERIAL PORT...!"

    def Stop(self):
        try:
            self.micro.write('S')
        except:
            print "UNABLE TO WRITE TO THE SERIAL PORT...!"

    def Go(self, LeftFront, RightFront, RightRear, LeftRear):
        self.boundsCheck(LeftFront, RightFront, LeftRear, RightRear)
        commandString = 'C' + str(LeftFront) + '|' + str(RightFront) + '|' + str(LeftRear) + '|' + str(RightRear) + '\n'
        try:
            self.micro.write(commandString)
        except:
            print "UNABLE TO WRITE TO THE SERIAL PORT...!"

    def boundsCheck(self, LF, RF, LR, RR):
        if (abs(LF) > 1.0 or abs(RF) > 1.0 or abs(LR) > 1.0 or abs(RR) > 1.0):
            print "***Exceded motor command bounds!***"
            raise BaseException
