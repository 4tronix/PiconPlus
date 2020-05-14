#! /usr/bin/env python
# Picon Plus Servo Test
# Use arrow keys to move servos on selected output
# Use number keys 0, 1, 2 to select servo
# Press Space Bar to centre selected servo
# Press Ctrl-C to stop
#

from __future__ import print_function
from piconplus import PiconPlus
import time

#======================================================================
# Reading single character by forcing stdin to raw mode
import sys
import tty
import termios

def readchar():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    if ch == '0x03':
        raise KeyboardInterrupt
    return ch

def readkey(getchar_fn=None):
    getchar = getchar_fn or readchar
    c1 = getchar()
    if ord(c1) != 0x1b:
        return c1
    c2 = getchar()
    if ord(c2) != 0x5b:
        return c1
    c3 = getchar()
    return chr(0x10 + ord(c3) - 65)  # 16=Up, 17=Down, 18=Right, 19=Left arrows

# End of single character reading
#======================================================================

pp = PiconPlus(0x24)
speed = 60

print ("Tests the servos by using the Up/Down arrow keys to control")
print ("Select servo with 0, 1, 2 number keys")
print ("Press <space> key to centre")
print ("Press Ctrl-C to end")
print

# Default to servo on Output Pin 0
servo = 0

# Set output mode to Servo on all Output Pins
pp.setOutputConfig(0, 2)
pp.setOutputConfig(1, 2)
pp.setOutputConfig(2, 2)

# Centre servo
servoVal = 90
pp.setOutput (servo, servoVal)

# main loop
try:
    while True:
        keyp = readkey()
        if keyp == 'w' or ord(keyp) == 16:
            servoVal = max (0, servoVal - 5)
            print ('Servo', servo,'Up', servoVal)
        elif keyp == 'z' or ord(keyp) == 17:
            servoVal = min (180, servoVal + 5)
            print ('Servo', servo, 'Down', servoVal)
        elif keyp == '0':
            servo = 0
            print ('Servo', servo)
        elif keyp == '1':
            servo = 1
            print ('Servo', servo)
        elif keyp == '2':
            servo = 2
            print ('Servo', servo)
        elif keyp == ' ':
            panVal = tiltVal = gripVal = 90
            print ('Servo', servo, 'Centre')
        elif ord(keyp) == 3:
            break
        pp.setOutput (servo, servoVal)

except KeyboardInterrupt:
    print()

finally:
    pp.cleanup()
    
