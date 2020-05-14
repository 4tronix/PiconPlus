# Python library for 4tronix Picon Zero
# Note that all I2C accesses are wrapped in try clauses with repeats
from __future__ import print_function
import smbus, time

bus = smbus.SMBus(1) # For revision 1 Raspberry Pi, change to bus = smbus.SMBus(0)

pzaddr = 0x26 # I2C address of Picon Zero

#---------------------------------------------
# Definitions of Commands to Picon Zero
MOTORA = 0
OUTCFG0 = 4
OUTPUT0 = 7
INCFG0 = 11
SETBRIGHT = 15
UPDATENOW = 16
RESET = 20
#---------------------------------------------

#---------------------------------------------
# General Constants
RETRIES = 10   # max number of retries for I2C calls
#---------------------------------------------

#---------------------------------------------
class PiconPlus:

#---------------------------------------------
# Get Version and Revision info
    def getRevision(self):
        for i in range(RETRIES):
            try:
                rval = bus.read_word_data (self.i2cAddress, 0)
                return [rval>>8, rval%256]
            except:
                if (self.debug):
                    print("Error in getRevision(), retrying")
                return [-1, -1]
#---------------------------------------------


#---------------------------------------------
# motor must be in range 0..3
# value must be in range -100 - +100
# values of -100 and +100 are treated as always ON,, no PWM
    def setMotor (self, motor, value):
        if (motor>=0 and motor<=3 and value>=-100 and value<100):
            for i in range(RETRIES):
                try:
                    bus.write_byte_data (self.i2cAddress, motor, value)
                    break
                except:
                    if (self.debug):
                        print("Error in setMotor(), retrying")

    def forward (self, speed):
        self.setMotor (0, speed)
        self.setMotor (1, speed)

    def reverse (self, speed):
        self.setMotor (0, -speed)
        self.setMotor (1, -speed)

    def spinLeft (self, speed):
        self.setMotor (0, -speed)
        self.setMotor (1, speed)

    def spinRight (self, speed):
        self.setMotor (0, speed)
        self.setMotor (1, -speed)

    def stop(self):
        self.setMotor (0, 0)
        self.setMotor (1, 0)

#---------------------------------------------

#---------------------------------------------
# Read data for selected input channel (analog or digital)
# Channel is in range 0 to 5
    def readInput (self, channel):
        if (channel>=0 and channel <=5):
            for i in range(RETRIES):
                try:
                    return bus.read_word_data (self.i2cAddress, channel + 1)
                except:
                    if (self.debug):
                        print("Error in readChannel(), retrying")
                    return -1

#---------------------------------------------

#---------------------------------------------
# Set configuration of selected output channel
# 0: On/Off, 1: PWM, 2: Servo
    def setOutputConfig (self, output, value):
        if (output>=0 and output<=2 and value>=0 and value<=2):
            for i in range(RETRIES):
                try:
                    bus.write_byte_data (self.i2cAddress, OUTCFG0 + output, value)
                    break
                except:
                    if (self.debug):
                        print("Error in setOutputConfig(), retrying")
#---------------------------------------------

#---------------------------------------------
# Set configuration of selected input channel
# 0: Digital, 1: Analog, 2: DS18B20
    def setInputConfig (self, channel, value, pullup = False):
        if (channel >= 0 and channel <= 3 and value >= 0 and value <= 2):
            if (value == 0 and pullup == True):
                value = 128
            for i in range(RETRIES):
                try:
                    bus.write_byte_data (self.i2cAddress, INCFG0 + channel, value)
                    break
                except:
                    if (self.debug):
                        print("Error in setInputConfig(), retrying")
#---------------------------------------------

#---------------------------------------------
# Set output data for selected output channel
# Mode  Name    Type    Values
# 0     On/Off  Byte    0 is OFF, 1 is ON
# 1     PWM     Byte    0 to 100 percentage of ON time
# 2     Servo   Byte    -100 to + 100 Position in degrees
    def setOutput (self, channel, value):
        if (channel>=0 and channel<=2):
            for i in range(RETRIES):
                try:
                    bus.write_byte_data (self.i2cAddress, OUTPUT0 + channel, value)
                    break
                except:
                    if (self.debug):
                        print("Error in setOutput(), retrying")
#---------------------------------------------

#---------------------------------------------
# Set the colour of an individual pixel (always output 5)
    def setPixel (self, Pixel, Red, Green, Blue, Update=True):
        pixelData = [Pixel, Red, Green, Blue]
        for i in range(RETRIES):
            try:
                bus.write_i2c_block_data (self.i2cAddress, Update, pixelData)
                break
            except:
                if (self.debug):
                    print("Error in setPixel(), retrying")

# Set the colour of all pixels
    def setAllPixels (self, Red, Green, Blue, Update=True):
        pixelData = [100, Red, Green, Blue]
        for i in range(RETRIES):
            try:
                bus.write_i2c_block_data (self.i2cAddress, Update, pixelData)
                break
            except:
                if (self.debug):
                    print("Error in setAllPixels(), retrying")

# Update the LEDs from the data buffer
    def updatePixels (self):
        for i in range(RETRIES):
            try:
                bus.write_byte_data (self.i2cAddress, UPDATENOW, 0)
                break
            except:
                if (self.debug):
                    print("Error in updatePixels(), retrying")

# Set the overall brightness of pixel array
    def setBrightness (self, brightness):
        for i in range(RETRIES):
            try:
                bus.write_byte_data (self.i2cAddress, SETBRIGHT, brightness)
                break
            except:
                if (self.debug):
                    print("Error in setBrightness(), retrying")
#---------------------------------------------

#---------------------------------------------
# Initialise the Board (same as cleanup)
    def __init__ (self, i2cAddress, debug=False):
        self.i2cAddress = i2cAddress
        self.debug = debug
        for i in range(RETRIES):
            try:
                bus.write_byte_data (self.i2cAddress, RESET, 0)
                break
            except:
                if (self.debug):
                    print("Error in init(), retrying")
        time.sleep(0.01)  #1ms delay to allow time to complete
        if (self.debug):
            print("Debug is", self.debug)
#---------------------------------------------

#---------------------------------------------
# Cleanup the Board (same as init)
    def cleanup (self):
        for i in range(RETRIES):
            try:
                bus.write_byte_data (self.i2cAddress, RESET, 0)
                break
            except:
                if (self.debug):
                    print("Error in cleanup(), retrying")
        time.sleep(0.001)   # 1ms delay to allow time to complete
#---------------------------------------------
