/*
 * ArduinoNunchuk - Improved Wii Nunchuk library for Arduino
 *
 * Copyright 2012-2013 Eddie Groshev
 *
 * Project URL: https://github.com/EdsterG/ArduinoNunchuk
 *
 * Based on the following projects/websites:
 *   http://www.gabrielbianconi.com/projects/arduinonunchuk/
 *   http://www.timteatro.net/2012/02/10/a-library-for-using-the-wii-nunchuk-in-arduino-sketches/
 */

#if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#include <Wire.h>
#include "ArduinoNunchuk.h"

// These are suitable defaults for most nunchuks, including knockoffs.
// If you intend to use the same nunchuk all the time and demand accu-
// racy, it is worth your time to measure these on your own.
// If you may want to use various nunchuks, you may want to
// calibrate using functions:
//   nunchuk_calibrate_joy()
//   nunchuk_calibrate_accelxy()
//   nunchuk_calibrate_accelz()
#define JOY_ZEROX 127 // Default center is most likely between 127-128
#define JOY_ZEROY 127
#define ACCEL_ZEROX 511 // Default center is around 511-512
#define ACCEL_ZEROY 511
#define ACCEL_ZEROZ 511

#define ADDRESS 0x52 //Address of the nunchuck

// Initialize and join the I2C bus, and tell the nunchuk we're
// talking to it. This function will work both with Nintendo
// nunchuks, or knockoffs.
void ArduinoNunchuk::init()
  {      
  Wire.begin();
  
  ArduinoNunchuk::_sendByte(0x55, 0xF0);
  ArduinoNunchuk::_sendByte(0x00, 0xFB); 
  
  ArduinoNunchuk::update();

  // Set default calibration centers
  _analogZeroX = JOY_ZEROX;
  _analogZeroY = JOY_ZEROY;
  _accelZeroX = ACCEL_ZEROX;
  _accelZeroY = ACCEL_ZEROY;
  _accelZeroZ = ACCEL_ZEROZ;
  }

// Gets data from the nunchuk and stores the raw values
// as instance variables.
void ArduinoNunchuk::update()
  { 
  int count = 0;      
  int values[6];
  uint8_t data[6];
  
  Wire.requestFrom (ADDRESS, 6); 
  
  while(Wire.available())
    {
    #if (ARDUINO >= 100)
      values[count] = Wire.read();
      data[count] = Wire.read();
    #else
      values[count] = Wire.receive();
      data[count] = Wire.receive();
    #endif
    count++;
    }
  
  // Unless you want to implement error checking (for failed read),
  // ignore the next two lines.
  //if (cnt < 5) // Haven't decided if I need to check for a failed read
    //return 0;  // from the controller. If need be I'll implement later.

  // Stores the raw x and y values of the the joystick, cast as ints.
  ArduinoNunchuk::analogX = values[0];
  ArduinoNunchuk::analogY = values[1];
/*
  Serial.print("int x: ");
  Serial.print(values[0]);
  Serial.print("byte x: ");
  Serial.print(data[0]);
  Serial.print("int y: ");
  Serial.print(values[1]);
  Serial.print("byte y: ");
  Serial.print(data[1]);
*/
  // Stores the raw 10-bit values from the 3-axis accelerometer sensor.
  // Of the six bytes recieved in a data payload from the nunchuk, bytes
  // 2, 3 and 4 are the most significant 8 bits of each 10-bit reading.
  // The final two bits are stored in the 6th bit along with the states
  // of the c and z button. These functions take the most significant
  // 8-bits and stacks it into a 16 bit unsigned integer, and then tacks
  // on the least significant bits from the 6th byte of the data
  // payload.
  // 
  // Load the most sig digits into a blank 16-bit unsigned int leaving
  // two bits in the bottom ( via a 2-bit shift, << 2) for the least sig
  // bits:
  //  0x0000 | nunchuk_buff[*] << 2
  // Add to the above, the least sig bits. The code for x:
  //  nunchuk_buf[5] & B00001100
  // for example selects the 3rd and 4th bits from the 6th byte of the
  // payload to be concatinated with nunchuk_buff[2] to complete the 10-
  // bit datum for a given axis.
  /*
  static inline uint16_t nunchuk_accelx()
    {
    return (0x0000 | (nunchuk_buf[2] << 2) +
      ((nunchuk_buf[5] & B00001100) >> 2));
    }

  static inline uint16_t nunchuk_accely()
    {
    return (0x0000 ^ (nunchuk_buf[3] << 2) +
      ((nunchuk_buf[5] & B00110000) >> 4));
    }

  static inline uint16_t nunchuk_accelz()
    {
    return (0x0000 ^ (nunchuk_buf[4] << 2) +
      ((nunchuk_buf[5] & B11000000) >> 6));
    }
  */
  ArduinoNunchuk::accelX = values[2] * 2 * 2 + ((values[5] >> 2) & 1) * 2 + ((values[5] >> 3) & 1);
  ArduinoNunchuk::accelY = values[3] * 2 * 2 + ((values[5] >> 4) & 1) * 2 + ((values[5] >> 5) & 1);
  ArduinoNunchuk::accelZ = values[4] * 2 * 2 + ((values[5] >> 6) & 1) * 2 + ((values[5] >> 7) & 1);

  // Stores c and z button states: 1 = pressed, 0 = not
  // The state is in the two least significant bits of the 6th byte.
  // In the data, a 1 is unpressed and 0 is pressed, so this will be
  // reversed. These functions use a bitwise AND to determine the value
  // and then the ! not operator to invert it and store
  // the appropriate state.
  ArduinoNunchuk::zButton = !((values[5] >> 0) & 1);
  ArduinoNunchuk::cButton = !((values[5] >> 1) & 1);
  
  ArduinoNunchuk::_sendByte(0x00, 0x00);
  }
  
void ArduinoNunchuk::_sendByte(byte data, byte location)
  {  
  Wire.beginTransmission(ADDRESS);
  
  #if (ARDUINO >= 100)
    Wire.write(location);
    Wire.write(data);  
  #else
    Wire.send(location);
    Wire.send(data); 
  #endif
  
  Wire.endTransmission();
  
  delay(10);
  }

// Uses port C (analog in) pins as power (pin3) & ground (pin2) for
// the nunchuk using a wiichuck adapter which can be made or bought.
void ArduinoNunchuk::wiiChuckPwr()
  {
  #define pwrpin PORTC3
  #define gndpin PORTC2
  DDRC |= _BV(pwrpin) | _BV(gndpin);
  PORTC &=~ _BV(gndpin);
  PORTC |=  _BV(pwrpin);
  delay(100); // wait for things to stabilize
  }

// Calibrate joystick so that we read the center position as (0,0).
// Otherwise, we use the default values from the header.
void ArduinoNunchuk::calibrate_joy()
  {
  _analogZeroX = analogX;
  _analogZeroY = analogY;
  // IMPLIMENT LATER: calibrate left/right/up/down normalized around 100
  // EXAMPLE: 100/abs(nunchuk.analogX-zeroJoyX)*2.55 = multiplier used in moveHead
  }

// Because gravity pulls down on the z-accelerometer while the nunchuk
// is upright, we need to calibrate {x,y} and {z} separately. Execute
// this function while the nunchuk is known to be upright and then 
// execute nunchuk_calibrate_accelz() when the nunchuk is on its side.
void ArduinoNunchuk::calibrate_accelxy()
  {
  _accelZeroX = accelX;
  _accelZeroY = accelY;
  }

// See documentation for nunchuk_calibrate_xy()
void ArduinoNunchuk::calibrate_accelz()
  {
  _accelZeroZ = accelZ;
  }

// Return calibrated x and y values of the joystick.
int ArduinoNunchuk::cAnalogX()
  {
  return analogX - _analogZeroX;
  }

int ArduinoNunchuk::cAnalogY()
  {
  return analogY - _analogZeroY;
  }

// Returns the x,y and z accelerometer values with calibration values
// subtracted.
int ArduinoNunchuk::cAccelX()
  {
    return accelX - _accelZeroX;
  }

int ArduinoNunchuk::cAccelY()
  {
    return accelY - _accelZeroY;
  }

int ArduinoNunchuk::cAccelZ()
  {
    return accelZ - _accelZeroZ;
  }

// Returns joystick angle in degrees. It uses the ratio of calibrated
// x and y potentiometer readings to find the angle, zero being direct
// right (positive x) and measured counter-clockwise from there.
//
// If the atan2 function returns a negative angle, it is rotated back
// into a positive angle. For those unfamiliar, the atan2 function
// is a more inteligent atan function which quadrant the vector <x,y>
// is in, and returns the appropriate angle.
int ArduinoNunchuk::joyAngle()
  {
  double theta;
  theta = atan2((double)ArduinoNunchuk::cAnalogY(),(double)cAnalogX());
  while (theta < 0) theta += 2*PI;
  return (int)(theta * RAD_TO_DEG);
  }

// Returns roll angle in degrees. Under the assumption that the
// only acceleration detected by the accelerometer is acceleration due
// to gravity, this function uses the ratio of the x and z
// accelerometer readings to gauge pitch. This only works while the
// nunchuk is being held still or at constant velocity with zero ext-
// ernal force.
int ArduinoNunchuk::rollAngle()
  {
  return (int)(atan2((double)cAccelX(),(double)cAccelZ()) * RAD_TO_DEG);
  }

// Returns pitch angle in degrees. Under the assumption that the
// only acceleration detected by the accelerometer is acceleration due
// to gravity, this function uses the ratio of the y and z
// accelerometer readings to gauge pitch.  This only works while the
// nunchuk is being held still or at constant velocity with zero ext-
// ernal force.
int ArduinoNunchuk::pitchAngle()
  {
  return (int)(atan2((double)cAccelY(),(double)cAccelZ()) * RAD_TO_DEG);
  }
