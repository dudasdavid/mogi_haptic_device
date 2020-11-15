#! /usr/bin/env python3

import time
import serial

from mogi_haptic_device.thread_wrapper import periodic

import rospy
from std_msgs.msg import String, Int16, Int32, Int16MultiArray



speed, steeringAngle,  = 0, 0
distance = 50

def robotCommThread():
    global speed, steeringAngle, distance
    
    
    if speed <= -50:
        speed = -50
        print("Speed saturation occured!")
    elif speed >= 50:
        speed = 50
        print("Speed saturation occured!")
        
    if steeringAngle < -50:
        steeringAngle = -50
        print("Angle saturation occured!")
    elif steeringAngle > 50:
        steeringAngle = 50
        print("Angle saturation occured!")
    
    speed2Send = speed + 50
    steeringAngle2Send = steeringAngle + 50
    
    
    command = "STR%03d" % (steeringAngle2Send)
    ser.write(b"%s\r\n" % command.encode('ascii','ignore'))
    
    time.sleep(0.01)
    
    command = "SPD%03d" % (speed2Send)
    ser.write(b"%s\r\n" % command.encode('ascii','ignore'))
    
    time.sleep(0.01)
    
    command = "DIS%03d" % (distance)
    ser.write(b"%s\r\n" % command.encode('ascii','ignore'))
    
    


def setSpeeds(data):
    global speed, steeringAngle, distance

    try:
        speed = data.data[0]
        steeringAngle = data.data[1]
        distance = data.data[2]

    except rospy.ROSInterruptException:
        pass

# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

ser.close()
ser.open()
ser.isOpen()

ser.write(b"SRE\r\n")
time.sleep(0.1)

rospy.init_node('rover_control_node', anonymous=True)
rospy.Subscriber('driver_control', Int16MultiArray, setSpeeds)

commThread = periodic(robotCommThread, 0.05, "Comm")  # 50ms
commThread.start()

input("Press any key to exit!")
commThread.exit()
ser.write(b"SRD\r\n")
