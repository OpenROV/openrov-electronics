/* Self-Test Code for OpenROV Controller Board 2.5

Initial Version 6 September 2013 W. Holm

*/

#include <Wire.h>

/* The following declarations are used for the I2C A/D converter on the test jig

     A/D Channels are as follows:
     Ch0 = 5V power
     Ch1 = 3.3V power
     Ch2 = Battery voltage, probed at PWM1+ pin
     
*/

const byte voltage5V = 1;
const byte voltage3V = 0;
const byte voltageBat = 2;

const int I2C_address = 0x48;  // I2C write address for A/D converter
const byte DAT[8] = {0x8C,0xCC,0x9C,0xDC,0xAC,0xEC,0xBC,0xFC};
                                 // Constant configuration data for A/D converter input mux
  

int adcOutput;    // temp used to hold 12-bit unsigned output of I2C A/D or 10-bit Mega A/D

/*  The following constants set the thresholds for the analog voltage checks on the I2C A/D converter
  The full-scale range is 0-4095 counts, corresponding to 0-2.5V
  The battery voltage (12.0V on test jig) is sensed through a 1:5 resistor divider.  Battery voltage should be
      2.0V +/-5%
  The 5V supply is sensed through a 1:2 resistor divider.  5V supply should be 1.666V +/- 5%
  3.3V supply is sensed through a 1:1 resistor divider.  3.3V supply should be 1.65V +/- 5%
*/

const int voltage5V_High = 2867;
const int voltage5V_Low = 2593;
const int voltage3V_High = 2838;
const int voltage3V_Low = 2568;
const int voltageBat_High = 3440;
const int voltageBat_Low = 3112;

/* The following constants set the thresholds for the analog current and voltage checks with the Mega A/D converter
  The full-scale range of the Mega A/D converter is 0-1023 counts, corresponding to 0-(the 5V supply voltage).
  The measured current draw on board 2.5A #001 has been between 6 and 20 counts.  For now, tolerance band on board current is 3-25.
*/

const int currentBoard_High = 25;
const int currentBoard_Low = 3;
//const int currentBat_High = xxx;
//const int currentBat_Low = xxx;
const int voltageMegaBat_High = 675;
const int voltageMegaBat_Low = 552;

void setup()
{
  Wire.begin();    // initialize Arduino Mega I2C port
  
  pinMode(13, OUTPUT);         // D13 LED
  pinMode(49, OUTPUT);          // D49 LED
  pinMode(1, OUTPUT);       // BB UART RX LED
  
  
}

void loop()
{
  
  /* Flash the LEDs for 1 second.  HIGH = LED on for 13 and 49, LOW for LED1 */
  digitalWrite(13, HIGH);
  digitalWrite(49, HIGH);
  digitalWrite(1, LOW); 
  delay(1000);
  digitalWrite(13, LOW);
  digitalWrite(49, LOW);
  digitalWrite(1, HIGH);
  delay(1000);
  
  /* Do a couple of dummy reads to clear the A/D */
  
  adcOutput = readADC(voltageBat);
  adcOutput = readADC(voltageBat);
  adcOutput = readADC(voltageBat); 
  
  /* Read the 12V test jig supply on the I2C A/D */

  adcOutput = readADC(voltageBat);
  if (adcOutput > voltageBat_High) blinkError(2); 
  if (adcOutput < voltageBat_Low) blinkError(2);

  /* Read the 5V supply on the I2C A/D */
  adcOutput = readADC(voltage5V);
  if (adcOutput > voltage5V_High) blinkError(3);
  if (adcOutput < voltage5V_Low) blinkError(3);

  /* Read the 3.3V supply on the I2C A/D */
  adcOutput = readADC(voltage3V);
  if (adcOutput > voltage3V_High) blinkError(4);
  if (adcOutput < voltage3V_Low) blinkError(4);

  /* Read the board current on the Mega A/D  */

  adcOutput = analogRead(0);
  if (adcOutput > currentBoard_High) blinkError(4);
  if (adcOutput < currentBoard_Low) blinkError(4);


  /* Read Battery1 current on the Mega A/D  */
 
  /* as a test, output the battery current to serial port #1 (D18) */
  Serial1.begin(9600);

  adcOutput = analogRead(6);
  Serial1.println(adcOutput);

  adcOutput = analogRead(6);
  Serial1.println(adcOutput);

  adcOutput = analogRead(6);
  Serial1.println(adcOutput);

  adcOutput = analogRead(6);
  Serial1.println(adcOutput);

  adcOutput = analogRead(6);
  Serial1.println(adcOutput);
 
 
 adcOutput = analogRead(6);
 
  /* Read Battery2 current on the Mega A/D  */

 adcOutput = analogRead(5);
 
  /*  Read battery voltage on the Mega A/D */

 adcOutput = analogRead(4);
 if (adcOutput > voltageMegaBat_High) blinkError(8);
 if (adcOutput < voltageMegaBat_Low) blinkError(8);
 
  /* Turn on PWM channels 1 (digital 44) and 2 (digital 45) and declare success */

  analogWrite(44, 128);
  analogWrite(45, 128);
  blinkSuccess();

}

 /* Read the desired channel from the A/D converter.  If the A/D converter does not respond, go to blink error #1 */
  
int readADC( byte channel)  // This function reads the appropriate channel of the A/D and returns the 12-bit value
{
  byte adval_High, adval_Low;    // Store A/D value (high byte, low byte)
  int adval;

  Wire.beginTransmission(I2C_address);
  Wire.write(DAT[channel]);        // Configure the device to read the called channel  
  Wire.endTransmission(); 
  delay(1);
    
  // Read A/D value
  Wire.requestFrom(I2C_address, 2);
  delay(100);
  if(Wire.available() == 2)          // Checkf for data from A/D converter
  { 
    adval_High = Wire.read();   // Receive A/D high byte
    adval_Low = Wire.read();    // Receive A/D low byte
    adval = (adval_High << 8) | adval_Low;
    return(adval);
  }
  else blinkError(1);
  return (-1);
  
}
  

void blinkSuccess()  // alternate LEDs 13 and 49 at 1 Hz overall rate
{
  while(1)
  {
    digitalWrite(13, LOW);
    digitalWrite(49, HIGH);
    delay(500); 
    digitalWrite(13, HIGH);
    digitalWrite(49, LOW);
    delay(500);
  }
}

void blinkError(int errNo)  // hold LED13 on, blink LED 49 to represent error number
{
  int count;
  digitalWrite(13, HIGH);
  while(1)
  {
    count = errNo;
    while(count > 0)
    {
      digitalWrite(49, HIGH);
      delay(250);
      digitalWrite(49, LOW);
      delay(250);
      count = count-1;
    }
    delay(1000);
  }
}
