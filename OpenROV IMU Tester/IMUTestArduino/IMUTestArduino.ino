// This file is a mix of the MPU9150Lib test code and Walt Homs MS5803 pressure sensor code


////////////////////////////////////////////////////////////////////////////
//
//  This file  part of MPU9150Lib
//
//  Copyright (c) 2013 Pansenti, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all 
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU9150Lib.h"
#include "CalLib.h"
#include <dmpKey.h>
#include <dmpmap.h>
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <EEPROM.h>

//  DEVICE_TO_USE selects whether the IMU at address 0x68 (default) or 0x69 is used
//    0 = use the device at 0x68
//    1 = use the device at ox69

//#define  DEVICE_TO_USE    0 // we read a switch to decide which address to use now

MPU9150Lib MPU;                                              // the MPU object

//  MPU_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the sensor data and DMP output

#define MPU_UPDATE_RATE  (20)

//  MAG_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the magnetometer data
//  MAG_UPDATE_RATE should be less than or equal to the MPU_UPDATE_RATE

#define MAG_UPDATE_RATE  (10)

//  MPU_MAG_MIX defines the influence that the magnetometer has on the yaw output.
//  The magnetometer itself is quite noisy so some mixing with the gyro yaw can help
//  significantly. Some example values are defined below:

#define  MPU_MAG_MIX_GYRO_ONLY          0                   // just use gyro yaw
#define  MPU_MAG_MIX_MAG_ONLY           1                   // just use magnetometer and no gyro yaw
#define  MPU_MAG_MIX_GYRO_AND_MAG       10                  // a good mix value 
#define  MPU_MAG_MIX_GYRO_AND_SOME_MAG  50                  // mainly gyros with a bit of mag correction 

//  MPU_LPF_RATE is the low pas filter rate and can be between 5 and 188Hz

#define MPU_LPF_RATE   40

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port

#define  SERIAL_PORT_SPEED  115200

// pressure sensor //
const int DevAddress = 0x76;  // 7-bit I2C address of the MS5803

// Here are the commands that can be sent to the 5803

const byte Reset = 0x1E;
const byte D1_256 = 0x40;
const byte D1_512 = 0x42;
const byte D1_1024 = 0x44;
const byte D1_2048 = 0x46;
const byte D1_4096 = 0x48;
const byte D2_256 = 0x50;
const byte D2_512 = 0x52;
const byte D2_1024 = 0x54;
const byte D2_2048 = 0x56;
const byte D2_4096 = 0x58;
const byte AdcRead = 0x00;
const byte PromBaseAddress = 0xA0;


unsigned int CalConstant[8];  // Matrix for holding calibration constants

long AdcTemperature, AdcPressure;  // Holds raw ADC data for temperature and pressure
float Temperature, Pressure, TempDifference, Offset, Sensitivity;
float T2, Off2, Sens2;  // Offsets for second-order temperature computation
 
byte ByteHigh, ByteMiddle, ByteLow;  // Variables for I2C reads

#define PIN_LED            13
#define PIN_LED_IMU_OK     6
#define PIN_LED_IMU_FAIL   7
#define PIN_LED_DEPTH_OK   4
#define PIN_LED_DEPTH_FAIL 5

#define PIN_SW_ADDRESS   11
#define PIN_BTN_SOFTRESET 12

boolean boardPresent = false; // is there a board present?
boolean testPass_imu = false; // imu works?
boolean testPass_depth = false; // depth sensor works?
boolean testPass_temp = false; //temperature sensor works?

void setup()
{
  boardPresent = false;
  testPass_imu = false;
  testPass_depth = false;
  testPass_temp = false;
  
  pinMode(PIN_SW_ADDRESS, INPUT);
  digitalWrite(PIN_SW_ADDRESS, HIGH);
  
  pinMode(PIN_BTN_SOFTRESET, INPUT);
  digitalWrite(PIN_BTN_SOFTRESET, HIGH);
  
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_LED_IMU_OK, OUTPUT);
  pinMode(PIN_LED_IMU_FAIL, OUTPUT);
  pinMode(PIN_LED_DEPTH_OK, OUTPUT);
  pinMode(PIN_LED_DEPTH_FAIL, OUTPUT);
  
  digitalWrite(PIN_LED, HIGH);
  digitalWrite(PIN_LED_IMU_OK, HIGH);
  digitalWrite(PIN_LED_IMU_FAIL, HIGH);
  digitalWrite(PIN_LED_DEPTH_OK, HIGH);
  digitalWrite(PIN_LED_DEPTH_FAIL, HIGH);
  delay(500);
  digitalWrite(PIN_LED, LOW);
  digitalWrite(PIN_LED_IMU_OK, LOW);
  digitalWrite(PIN_LED_IMU_FAIL, LOW);
  digitalWrite(PIN_LED_DEPTH_OK, LOW);
  digitalWrite(PIN_LED_DEPTH_FAIL, LOW);
  
  Serial.begin(SERIAL_PORT_SPEED);
    Serial.println("Program start");
    
    /*
    while(!Serial.available())
    {
      Serial.println("Press any key to continue");
      delay(500);
    }*/
  
  Wire.begin();
  sendCommand(Reset);
  delay(10);
  delay(1000);
  Serial.println("Device is reset");
   
  // Get the calibration constants and store in array (for pressure sensor)

  
  for (byte i = 0; i < 8; i++)
  {
    sendCommand(PromBaseAddress + (2*i));
    Wire.requestFrom(DevAddress, 2);
    while(Wire.available()){
      ByteHigh = Wire.read();
      ByteLow = Wire.read();
    }
    CalConstant[i] = (((unsigned int)ByteHigh << 8) + ByteLow);
  }

}

void loop2() // test loop to swap with main loop for quick trials
{
  Serial.println("Test");
  digitalWrite(PIN_LED_IMU_OK, digitalRead(PIN_BTN_SOFTRESET));
}

void loop()
{  
  boolean addrSwitchPos = digitalRead(PIN_SW_ADDRESS); // keep track so we can reset if this is changed after testing  
  float pressure = 0, temp = 0;

  // test IMU
  MPU.selectDevice(addrSwitchPos);                         // only needed if device has changed since init but good form anyway
  if(MPU.init(MPU_UPDATE_RATE, MPU_MAG_MIX_GYRO_AND_MAG, MAG_UPDATE_RATE, MPU_LPF_RATE))   // start the MPU
    testPass_imu = true; // pass if we can initialize, just show we are talking to the imu
  
  // test pressure
  pressure = getPressure();
  if((pressure<1) && (pressure > -1))
      testPass_depth = true;
  
  // test temperature
  temp = getTemp();
  if(temp > 0)
    if(temp < 100)
      if(temp != 20) // this seems to be the value with no sensor connected
        testPass_temp = true;
  Serial.print("Temp: ");
  Serial.println(temp);
  
  // results
  Serial.println("--------------------------------");
  Serial.println("Test results");
  Serial.print("IMU  : ");
  if(testPass_imu)
  {
    digitalWrite(PIN_LED_IMU_OK, HIGH);    
    Serial.println("PASS");
  }
  else
  {
    digitalWrite(PIN_LED_IMU_FAIL, HIGH);    
    Serial.println("FAIL");
  }
  Serial.print("DEPTH: ");
  if(testPass_depth)
  {
    digitalWrite(PIN_LED_DEPTH_OK, HIGH);
    Serial.println("PASS");
  }
  else
  {    
    digitalWrite(PIN_LED_DEPTH_FAIL, HIGH);
    Serial.println("FAIL");  
  }
  Serial.print("TEMP : ");
  if(testPass_temp)
    Serial.println("PASS");
  else
    Serial.println("FAIL");
  
  Serial.println("--------------------------------");
  // tests complete
  
    
  // get stuck until a new board is inserted
  Serial.println("Test complete, press any key to retest or use hardware soft reset button.");
  while(true)
  {
    
    if(Serial.available()) break;
    if(!digitalRead(PIN_BTN_SOFTRESET)) break;
    if(digitalRead(PIN_SW_ADDRESS) != addrSwitchPos) break;
    if(testPass_imu && testPass_temp && testPass_depth)
    {
      digitalWrite(PIN_LED, HIGH);
      delay(100);
    }
    else
    {
    digitalWrite(PIN_LED, HIGH);
      delay(250);
      digitalWrite(PIN_LED, LOW);
      delay(250);
    }

  }
  
  setup();
  
}

float getPressure()
{
  sendCommand(D1_512);
  delay(10);
  sendCommand(AdcRead);
  Wire.requestFrom(DevAddress, 3);
  while(Wire.available())
  {
    ByteHigh = Wire.read();
    ByteMiddle = Wire.read();
    ByteLow = Wire.read();
  }
  AdcPressure = ((long)ByteHigh << 16) + ((long)ByteMiddle << 8) + (long)ByteLow;
  
  // Calculate the pressure parameters
  
  Offset = (float)CalConstant[2] * pow(2,16);
  Offset = Offset + ((float)CalConstant[4] * TempDifference / pow(2, 7));

  Sensitivity = (float)CalConstant[1] * pow(2, 15);
  Sensitivity = Sensitivity + ((float)CalConstant[3] * TempDifference / pow(2, 8));
  
  // Add second-order corrections
  
  Offset = Offset - Off2;
  Sensitivity = Sensitivity - Sens2;
  
  // Calculate absolute pressure in bars

  Pressure = (float)AdcPressure * Sensitivity / pow(2, 21);
  Pressure = Pressure - Offset;
  Pressure = Pressure / pow(2, 15);
  Pressure = Pressure / 10000;  // Set output to bars;
  
  // Convert to psig and display
  
  Pressure = Pressure - 1.015;  // Convert to gauge pressure (subtract atmospheric pressure)
  Pressure = Pressure * 14.50377;  // Convert bars to psi
  Serial.print("Pressure in psi is: ");
  Serial.println(Pressure);
  Serial.println();
  
  return Pressure;
}

float getTemp()
{
  sendCommand(D2_512);
  delay(10);
  sendCommand(AdcRead);
  Wire.requestFrom(DevAddress, 3);
  while(Wire.available())
  {
    ByteHigh = Wire.read();
    ByteMiddle = Wire.read();
    ByteLow = Wire.read();
  }
  AdcTemperature = ((long)ByteHigh << 16) + ((long)ByteMiddle << 8) + (long)ByteLow;
 // Serial.print("D2 is: ");
//  Serial.println(AdcTemperature);
  
    
  // Calculate the Temperature (first-order computation)
  
  TempDifference = (float)(AdcTemperature - ((long)CalConstant[5] << 8));
  Temperature = (TempDifference * (float)CalConstant[6])/ pow(2, 23);
  Temperature = Temperature + 2000;  // This is the temperature in hundredths of a degree C
  
  // Calculate the second-order offsets
  
  if (Temperature < 2000.0)  // Is temperature below or above 20.00 deg C ?
  {
    T2 = 3 * pow(TempDifference, 2) / pow(2, 33);
    Off2 = 1.5 * pow((Temperature - 2000.0), 2);
    Sens2 = 0.625 * pow((Temperature - 2000.0), 2);
  }
  else
  {
    T2 = (TempDifference * TempDifference) * 7 / pow(2, 37);
    Off2 = 0.0625 * pow((Temperature - 2000.0), 2); 
    Sens2 = 0.0;
  }
  
  // Check print the offsets
  
  Serial.println("Second-order offsets are:");
  Serial.println(T2);
  Serial.println(Off2);
  Serial.println(Sens2);
  
  
  // Print the temperature results
  
  Temperature = Temperature / 100;  // Convert to degrees C
  Serial.print("First-Order Temperature in Degrees C is ");
  Serial.println(Temperature);
  Serial.print("Second-Order Temperature in Degrees C is ");
  Serial.println(Temperature - (T2 / 100));
  
  return Temperature;
}

void sendCommand(byte command){
  Wire.beginTransmission(DevAddress);
  Wire.write(command);
  Wire.endTransmission();
}
