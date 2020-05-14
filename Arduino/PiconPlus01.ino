/* Picon PLUS CONTROL
-
- 4 motors driven by H-Bridges: (4 PWM pins, 4 Direction pins)
- 3 general purpose outputs: Can be LOW, HIGH, PWM or Servo
- 1 dedicated Fireled (WS2812B) pin
- 4 general purpose inputs: Can be Analog or Digital
-
- Each I2C packet comprises a Register/Command Pair of bytes

Write Only Registers
--------------------
Register  Name      Type  Values
0   Motor0_Data     Byte  -100 (full reverse) to +100 (full forward)
1   Motor1_Data     Byte  -100 (full reverse) to +100 (full forward)
2   Motor2_Data     Byte  -100 (full reverse) to +100 (full forward)
3   Motor3_Data     Byte  -100 (full reverse) to +100 (full forward)
4   Output0_Config  Byte  0: On/Off, 1: PWM, 2: Servo
5   Output1_Config  Byte  0: On/Off, 1: PWM, 2: Servo
6   Output2_Config  Byte  0: On/Off, 1: PWM, 2: Servo
7   Output0_Data    Byte  Data value(s)
8   Output1_Data    Byte  Data value(s)
9   Output2_Data    Byte  Data value(s)
10  Fireled_Data    4 Bytes 0:Pixel ID, 1:Red, 2:Green, 3:Blue
11  Input0_Config   Byte  0: Digital, 1:Analog, 2:DS18B20  (NB. 0x80 is Digital input with pullup)
12  Input1_Config   Byte  0: Digital, 1:Analog, 2:DS18B20  (NB. 0x80 is Digital input with pullup)
13  Input2_Config   Byte  0: Digital, 1:Analog, 2:DS18B20  (NB. 0x80 is Digital input with pullup)
14  Input3_Config   Byte  0: Digital, 1:Analog, 2:DS18B20  (NB. 0x80 is Digital input with pullup)
15  Set Brightness  Byte  0..255. Scaled max brightness (default is 40)
16  Update Pixels   Byte  dummy value - forces updating of neopixels
20  Reset           Byte  dummy value - resets all values to initial state

Read Only Registers - These are WORDs
-------------------------------------
Register  Name  Type  Values
0 Revision      Word  Low Byte: Firmware Build, High Byte: PCB Revision
1 Input0_Data   Word  0 or 1 for Digital, 0..1023 for Analog
2 Input1_Data   Word  0 or 1 for Digital, 0..1023 for Analog
3 Input2_Data   Word  0 or 1 for Digital, 0..1023 for Analog
4 Input3_Data   Word  0 or 1 for Digital, 0..1023 for Analog
5 PSU_Data      Word  PSU Voltage (mV)
6 I2C_Data      Word  Voltage on I2C Setting Pin (mV)

Data Values for Output Data Registers
--------------------------------------
Mode  Name    Type    Values
0     On/Off  Byte    0 is OFF, 1 is ON
1     PWM     Byte    0 to 100 percentage of ON time
2     Servo   Byte    -100 to + 100 Position in degrees

*/

/* Rev08: Adds Pullup option for Digital and DS18B20 inputs
*/

#include "FastLED.h"
#include <Wire.h>
#include <OneWire.h>
#include <Servo.h>

#define DEBUG         false
#define BOARD_REV     4   // Board ID for PiconPlus
#define FIRMWARE_REV  1   // Firmware Revision

// I2C Command Set
#define MOTOR0_DATA   0
#define MOTOR1_DATA   1
#define MOTOR2_DATA   2
#define MOTOR3_DATA   3
#define OUTPUT0_CFG   4
#define OUTPUT1_CFG   5
#define OUTPUT2_CFG   6
#define OUTPUT0_DATA  7
#define OUTPUT1_DATA  8
#define OUTPUT2_DATA  9
#define FIRELED_DATA  10
#define INPUT0_CFG    11
#define INPUT1_CFG    12
#define INPUT2_CFG    13
#define INPUT3_CFG    14
#define SET_BRIGHT    15
#define UPDATE_NOW    16
#define RESET         20

// Output Config Values
#define CFGONOFF  0
#define CFGPWM    1
#define CFGSERVO  2

// Input Config Values
#define CFGDIG    0
#define CFGDIGPU  0x80
#define CFGANA    1
#define CFG18B20  2

// Test Pins
#define I2CSET A7
#define PSUPIN A6

// Fireleds
#define LEDPIN 13
#define NUMLEDS 100
#define BRIGHTNESS 40
CRGB leds[NUMLEDS];
bool doShow = false;
int lastPSU = -1;

// Motors
#define NUMMOTORS 4
byte motorPwm[NUMMOTORS] = {9, 6, 5, 3};  // PWM pins for motors
byte motorDir[NUMMOTORS] = {8, 7, 4, 2};  // Direction pins for motors

// Outputs
#define NUMOUTPUTS 3
byte outputPins[NUMOUTPUTS] = {10, 11, 12}; // Output pins
byte outputConfigs[NUMOUTPUTS] = {0, 0, 0};  // 0: On/Off, 1: PWM, 2: Servo
Servo servos[NUMOUTPUTS];  // every output is potentially a servo

// PWM runs from 0 to 100 with 0 being fully OFF and 100 being fully ON
byte pwm[NUMOUTPUTS] = {0, 0, 0};  // array of PWM values, one value for each motor/output. Value is the point in the cycle that the PWM pin is switched OFF, with it being switch ON at 0. 100 means always ON: 0 means always OFF
byte pwmcount = 0;

// Inputs
#define NUMINPUTS 4
byte inputPins[NUMINPUTS] = {A0, A1, A2, A3}; // Input pins
byte inputConfigs[NUMINPUTS] = {0, 0, 0, 0};    // 0: Digital, 1:Analog
int inputValues[NUMINPUTS+2]; // store analog input values (words) including test pins

// DS18S20 Temperature chip i/o
OneWire ds0(A0);  // on pin A0
OneWire ds1(A1);  // on pin A1
OneWire ds2(A2);  // on pin A2
OneWire ds3(A3);  // on pin A3

// Global address data for DS18B20 input devices
byte B20_addr0[8];
byte B20_addr1[8];
byte B20_addr2[8];
byte B20_addr3[8];

// I2C
int i2cAddress;
byte inputChannel = 0; // selected reading channel

void setup()
{
  if (DEBUG)
  {
    Serial.begin(115200);
    Serial.println("Starting...");
    delay(1000);
  }
  i2cAddress = getI2c();
  if (DEBUG)
  {
    delay(1000);
    Serial.println("I2C: 0x" + String(i2cAddress, HEX));
    delay(1000);
  }
  Wire.begin(i2cAddress);         // join i2c bus with defined address
  Wire.onRequest(requestEvent);   // register event
  Wire.onReceive(receiveEvent);   // register wire.write interrupt event
  for (int i = 0; i < NUMMOTORS; i++)
  {
    pinMode (motorPwm[i], OUTPUT);
    pinMode (motorDir[i], OUTPUT);
  }
  for (int i = 0; i < NUMOUTPUTS; i++)
  {
    pinMode (outputPins[i], OUTPUT);
  }

  FastLED.addLeds<WS2812B, LEDPIN, GRB>(leds, NUMLEDS);
  FastLED.setBrightness(BRIGHTNESS);

  resetAll(); // initialise all I/O to safe values
}

// Reset routine should be called at start and end of all programs to ensure that board is set to correct configuration. Python library routines init() and cleanup() will action this
void resetAll()
{
  //disconnect all servos
  for (int i=0; i<NUMOUTPUTS; i++)
    servos[i].detach();
  //clear all Fireleds
  allOff();  
  // set all inputs to Digital
  for (int i=0; i<NUMINPUTS; i++)
  {
    setInCfg(i, CFGDIG); //Call input config to ensure inputs are properly reset.
  }
  // set all outputs to On/Off
  for (int i=0; i<NUMOUTPUTS; i++)
    outputConfigs[i] = CFGONOFF;
  lastPSU = -1;
}

int getI2c()
{
  int iset = analogRead(I2CSET);
  if (iset < 470)
    i2cAddress = 0x27;
  else if (iset < 628)
    i2cAddress = 0x26;
  else if (iset > 885)
    i2cAddress = 0x25;
  else
    i2cAddress = 0x24;
}

// The main loop handles the PWM and reading of inputs. Counts from 1..100 and matches the count value with the pwm values. If the same then the signal goes from Low to High
void loop()
{
  //Serial.println("Looping...");
  if (pwmcount == 0)
  {
    for (int i=0; i<NUMOUTPUTS; i++)  // Check all outputs for PWM
      if (pwm[i]>0 && pwm[i]<100)   // PWM values of 0 or 100 means no PWM, so never change this pin
      {
        //Serial.println("High:" + String(outputPins[i]));
        digitalWrite (outputPins[i], HIGH);
      }
  }
  else
  {
    for (int i=0; i<NUMOUTPUTS; i++)
      if (pwm[i] == pwmcount)
      {
        //Serial.println("Low:" + String(outputPins[i]));
        digitalWrite (outputPins[i], LOW);
      }
  }
  //delay(10);
  delayMicroseconds(10);
  if (++pwmcount > 99)  // as pwmcount never goes over 99, then a value of 100+ will never be changed to LOW so will be on permanently
  {
    int newPSU=0;
    pwmcount = 0;
    //Serial.println("Tick");
    for (int i=0; i<NUMINPUTS; i++)
    {
      switch (inputConfigs[i])
      {
        case CFGDIG:
        case CFGDIGPU: inputValues[i] = digitalRead(inputPins[i]); break;
        case CFGANA: inputValues[i] = analogRead(inputPins[i]); break;
        case CFG18B20: startConversion(i); inputValues[i] = getTemp(i); break; // data read is from previous conversion as it can take up to 750ms
      }
    }
    inputValues[NUMINPUTS+1] = voltageRead(I2CSET, 5000);
    inputValues[NUMINPUTS] = voltageRead(PSUPIN, 28000);
    if(inputValues[NUMINPUTS] > 7200)
       newPSU = 7200;
    else if (inputValues[NUMINPUTS] > 6600)
       newPSU = 6600;
    //Serial.println("PSU: " + String(inputValues[NUMINPUTS]) + "  Q: " + String(newPSU));
    if (newPSU != lastPSU)
    {
      lastPSU = newPSU;
      if(lastPSU == 7200)
        leds[0] = CRGB(0, 255, 0);
      else if(lastPSU == 6600)
        leds[0] = CRGB(255, 255, 0);
      else
        leds[0] = CRGB(255, 0, 0);
      doShow = true;
    }
    if (doShow)
    {
      FastLED.show();
      doShow = false;
    }
    if (false)
      Serial.println("pwm0:" + String(pwm[0]) + "  pwm1:" + String(pwm[1]));
  }
}

int voltageRead(int pin, long fsd)
{
  long rval;
  rval = (analogRead(pin)*fsd)/1023L;
  return(rval);
}

// This function is called for every data read request. We always return a Word (16-bits). Low byte first.
void requestEvent()
{
  byte outBuf[2];
  if (inputChannel == 0)
  {
    outBuf[0] = BOARD_REV;
    outBuf[1] = FIRMWARE_REV;
  }
  else
  {
    outBuf[0] = inputValues[inputChannel-1]&0xff;
    outBuf[1] = inputValues[inputChannel-1]>>8;
  }
  Wire.write(outBuf, 2);
}

// function that executes whenever data is received from master
void receiveEvent(int count)
{
  //return;
  if (DEBUG)
    Serial.println("Data count:" + String(count));
  if (count == 1) // Read request register
  {
    inputChannel = Wire.read();
    if (DEBUG)
      Serial.println("Channel:" + String(inputChannel));
  }
  else if (count == 2)
  {
    byte regSel = Wire.read();
    byte regVal = Wire.read();
    if (DEBUG)
      Serial.println("Register:" + String(regSel) + "  Value:" + String(regVal));
    switch(regSel)
    {
      case MOTOR0_DATA: setMotor(0, regVal); break;
      case MOTOR1_DATA: setMotor(1, regVal); break;
      case MOTOR2_DATA: setMotor(2, regVal); break;
      case MOTOR3_DATA: setMotor(3, regVal); break;
      case OUTPUT0_CFG: setOutCfg(0, regVal); break;
      case OUTPUT1_CFG: setOutCfg(1, regVal); break;
      case OUTPUT2_CFG: setOutCfg(2, regVal); break;
      case OUTPUT0_DATA: setOutData(0, regVal); break;
      case OUTPUT1_DATA: setOutData(1, regVal); break;
      case OUTPUT2_DATA: setOutData(2, regVal); break;
      case INPUT0_CFG: setInCfg(0, regVal); break;
      case INPUT1_CFG: setInCfg(1, regVal); break;
      case INPUT2_CFG: setInCfg(2, regVal); break;
      case INPUT3_CFG: setInCfg(3, regVal); break;
      case SET_BRIGHT: FastLED.setBrightness(regVal); break;
      case UPDATE_NOW: doShow = true; break;
      case RESET: resetAll(); break;
    }
  }
  else if (count == 5)
  {
    byte updates = Wire.read();
    byte pixel = Wire.read();
    byte red = Wire.read();
    byte green = Wire.read();
    byte blue = Wire.read();
    //if (DEBUG)
      //Serial.println("Reg:Val::" + String(updates) + ":" + String(pixel) + "  Red:" + String(red) + "  Green:" + String(green) + "  Blue:" + String(blue));
    if (pixel < NUMLEDS)
    {
      leds[pixel+1] = CRGB(red, green, blue);
      if (updates != 0)
        doShow = true;
    }
    else if (pixel == NUMLEDS)  // special case meaning ALL pixels
    {
      for (int i=1; i<(NUMLEDS+1); i++)
        leds[i] = CRGB(red, green, blue);
      if (updates != 0)
        doShow = true;
    }
  }
  else // something odd happened. Read all outstanding bytes
  {
    if (DEBUG)
      Serial.println("Odd count:" + String(count));
    for (int i=0; i<count; i++)
      Wire.read();
  }
}

// Set new Motor Value
void setMotor(int motor, int mSpeed)
{
  analogWrite(motorPwm[motor], 0); // set to zero to stop glitches
  digitalWrite(motorDir[motor], (mSpeed>127) ? 1 : 0);
  if (mSpeed > 127)
    mSpeed = 256 - mSpeed;
  if (DEBUG)
    Serial.println("Mspeed: " + String(map(mSpeed, 0, 100, 0, 1023)));
  analogWrite(motorPwm[motor], map(mSpeed, 0, 100, 0, 255));
}

// Set configuration for output pins 0: On/Off, 1: PWM, 2: Servo
void setOutCfg(byte outReg, byte value)
{
  if (DEBUG)
    Serial.println("setOutCfg: " + String(outReg) + ", " + String(value));
  if (outputConfigs[outReg] == value)
    return; // don't attach same servo twice, or even set the same value as it currently is
  if (outputConfigs[outReg] != CFGSERVO && value == CFGSERVO)
    servos[outReg].attach(outputPins[outReg]);
  else if (outputConfigs[outReg] == CFGSERVO && value != CFGSERVO)
    servos[outReg].detach();
  outputConfigs[outReg] = value;
}

void setOutData(byte outReg, byte value)
{
  switch (outputConfigs[outReg])
  {
    case CFGONOFF:
      pwm[outReg + 4] = 0;
      if(value == 0)
        digitalWrite(outputPins[outReg], LOW);
      else
        digitalWrite(outputPins[outReg], HIGH);
      break;
    case CFGPWM:
      if (value == 0)
      {
        pwm[outReg + 4] = 0;
        digitalWrite(outputPins[outReg], LOW);
      }
      else if (value >= 100)
      {
        pwm[outReg + 4] = 0;
        digitalWrite(outputPins[outReg], HIGH);
      }
      else
      {
        pwm[outReg + 4] = min(value, 99);
        digitalWrite(outputPins[outReg], LOW); // not strictly necessary as PWM will kick in eventually
      }
      break;
    case CFGSERVO:
      servos[outReg].write(value);
      break;
  }
}

void setInCfg(byte inReg, byte value)
{
  if (DEBUG)
  {
    Serial.println("inReg:" + String(inReg) + "  value:" + String(value));
  }
  inputConfigs[inReg] = value;
  if (value == CFGDIGPU) 
  {
    pinMode(inputPins[inReg], INPUT_PULLUP);
  }
  else
  {
    pinMode(inputPins[inReg], INPUT);
  }
  if (value == CFG18B20)
  {
    switch (inReg)
    {
      case 0: ds0.reset_search(); ds0.search(B20_addr0); break;
      case 1: ds1.reset_search(); ds1.search(B20_addr1); break;
      case 2: ds2.reset_search(); ds2.search(B20_addr2); break;
      case 3: ds3.reset_search(); ds3.search(B20_addr3); break;
    }
  }
}

// DS18B20 Temperature Sensor Reading
int getTemp(int index)
{
  byte data[12];
  switch (index)
  {
    case 0:
      ds0.reset();
      ds0.select(B20_addr0);
      ds0.write(0xBE);         // Read Scratchpad
      for ( int i = 0; i < 9; i++)
        data[i] = ds0.read();
      break;
    case 1:
      ds1.reset();
      ds1.select(B20_addr1);
      ds1.write(0xBE);         // Read Scratchpad
      for ( int i = 0; i < 9; i++)
        data[i] = ds1.read();
      break;
    case 2:
      ds2.reset();
      ds2.select(B20_addr2);
      ds2.write(0xBE);         // Read Scratchpad
      for ( int i = 0; i < 9; i++)
        data[i] = ds2.read();
      break;
    case 3:
      ds3.reset();
      ds3.select(B20_addr3);
      ds3.write(0xBE);         // Read Scratchpad
      for ( int i = 0; i < 9; i++)
        data[i] = ds3.read();
      break;
  }
  return data[1]*256 + data[0];
}

void startConversion(int index)
{
  switch (index)
  {
    case 0: ds0.reset(); ds0.select(B20_addr0); ds0.write(0x44,0); break;
    case 1: ds1.reset(); ds1.select(B20_addr1); ds1.write(0x44,0); break;
    case 2: ds2.reset(); ds2.select(B20_addr2); ds2.write(0x44,0); break;
    case 3: ds3.reset(); ds3.select(B20_addr3); ds3.write(0x44,0); break;
  }
}



// Set single LED value
void setLED(int ID, int red, int green, int blue)
{
  leds[ID] = CRGB(red, green, blue);
  FastLED.show();
}

// Turns all the LEDs to OFF
void allOff()
{
  for (int i=0; i<NUMLEDS; i++)
    leds[i] = 0;
  FastLED.show();
}

// Sets all the LEDs to the same colour
void setAll(int red, int green, int blue)
{
  for (int i=0; i<NUMLEDS; i++)
    leds[i] = CRGB(red, green, blue);
  FastLED.show();
}

