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
#define UPFACTOR 1    // These factors are used to normalize the
#define DOWNFACTOR 1  // joystick values for UP, DOWN, LEFT, and RIGHT
#define LEFTFACTOR 1  // to have amaximum magnitude of 100
#define RIGHTFACTOR 1

#define ADDRESS 0x52 //Address of the nunchuck

// Initialize and join the I2C bus, and tell the nunchuk we're
// talking to it. This function will work both with Nintendo
// nunchuks, or knockoffs.
void ArduinoNunchuk::init(bool wiichuck){
  if (wiichuck)
    _wiiChuckPwr();

  Wire.begin();

  ArduinoNunchuk::_sendByte(0x55, 0xF0);
  delay(7.5);
  ArduinoNunchuk::_sendByte(0x00, 0xFB);
  delay(7.5);

  ArduinoNunchuk::update();

  // Calibrate joy center automatically on start up
  ArduinoNunchuk::calibrate_joy("center");
  // See documentation for calibrate_accelxy()
  // These values can NOT be calibrated automatically
  _accelZeroX = ACCEL_ZEROX;
  _accelZeroY = ACCEL_ZEROY;
  _accelZeroZ = ACCEL_ZEROZ;
  _upFactor = UPFACTOR;
  _downFactor = DOWNFACTOR;
  _leftFactor = LEFTFACTOR;
  _rightFactor = RIGHTFACTOR;
  ArduinoNunchuk::personalCalibrationValues();
}

// Gets data from the nunchuk and stores the raw values
// as instance variables.
void ArduinoNunchuk::update(){ 
  int count = 0;      
  int values[6];
  
  Wire.requestFrom (ADDRESS, 6); 
  
  while(Wire.available()){
    #if (ARDUINO >= 100)
      values[count] = Wire.read();
    #else
      values[count] = Wire.receive();
    #endif
    count++;
  }
  
  // Unless you want to implement error checking (for failed read),
  // ignore the next two lines.
  //if (cnt < 5) // Haven't decided if I need to check for a failed read
    //return 0;  // from the controller. If need be I'll implement later.

  // Stores the raw x and y values of the the joystick.
  analogX = values[0];
  analogY = values[1];

  // Process the raw 10-bit values from the 3-axis accelerometer sensor.
  // Of the six bytes recieved in a data payload from the nunchuk, bytes
  // 2, 3 and 4 are the most significant 8 bits of each 10-bit reading.
  // The final two bits are stored in the 6th bit along with the states
  // of the c and z button. These assignments take the most significant
  // 8-bits shift them and then tack on the least significant bits from
  // the 6th byte of the data payload.
  // 
  // Shift the most significant bits twice, the next significant once,
  // and finally adds the least significant, the next significant, and
  // the most significat digits to get the final answer.
  // (EX: If we had a number, lets say 23, and it was the most significant
  // part of the final answer we would multiply it by 10 twice to get
  // 2300, then we would take 4 and multiply it by 10 once to get 40,
  // then we would take 1 and leave it the way it is. Finally we add
  // them all together, 2300+40+1 to get 2341. Since the binary system
  // is of base 2, every time we multiply by 2, it gives us a similar
  // effect in binary digits as multiplying by 10 in decimal digits.)
  accelX = values[2] * 2 * 2 + ((values[5] >> 2) & 1) * 2 + ((values[5] >> 3) & 1);
  accelY = values[3] * 2 * 2 + ((values[5] >> 4) & 1) * 2 + ((values[5] >> 5) & 1);
  accelZ = values[4] * 2 * 2 + ((values[5] >> 6) & 1) * 2 + ((values[5] >> 7) & 1);
  // Stores c and z button states: 1 = pressed, 0 = not
  // The state is in the two least significant bits of the 6th byte.
  // In the data, a 1 is unpressed and 0 is pressed, so this will be
  // reversed. These functions use a bitwise AND to determine the value
  // and then the ! not operator to invert it and store
  // the appropriate state.
  zButton = !((values[5] >> 0) & 1);
  cButton = !((values[5] >> 1) & 1);
  
  ArduinoNunchuk::_sendByte(0x00, 0x00);
  _checkButtons();
}
  
void ArduinoNunchuk::_sendByte(byte data, byte location){  
  Wire.beginTransmission(ADDRESS);
  
  #if (ARDUINO >= 100)
    Wire.write(location);
    Wire.write(data);  
  #else
    Wire.send(location);
    Wire.send(data); 
  #endif
  
  Wire.endTransmission();
  delay(2.5); // This delay used to be 10, but it interfered with time
              // sensitive parts of my program so I reduced it to 2.5
}

// Uses port C (analog in) pins as power (pin3) & ground (pin2) for
// the nunchuk using a wiichuck adapter which can be made or bought.
void ArduinoNunchuk::_wiiChuckPwr(){
  #define pwrpin PORTC3
  #define gndpin PORTC2
  DDRC |= _BV(pwrpin) | _BV(gndpin);
  PORTC &=~ _BV(gndpin);
  PORTC |=  _BV(pwrpin);
  delay(100); // wait for things to stabilize
}

// Calibrate joystick so that we read the center position as (0,0).
// Otherwise, we use the default values from the header.
void ArduinoNunchuk::calibrate_joy(char location[]){
  ArduinoNunchuk::update();
  if (location == "center" && analogX<142 && analogX>112 && analogY<142 && analogY>112){
    // This long if statement is used make sure the nunchuck isn't
    // calibrated when the joystick is accidenly pushed to one of the
    // edges. It might not be needed, in that even I'll remove this check
    _analogZeroX = analogX;
    _analogZeroY = analogY;
  }
  else if (location == "up" && cAnalogY()>85)
    _upFactor=abs(100/(float)cAnalogY());
  else if (location == "down" && cAnalogY()<-85)
    _downFactor=abs(100/(float)cAnalogY());
  else if (location == "left" && cAnalogX()<-85)
    _leftFactor=abs(100/(float)cAnalogX());
  else if (location == "right" && cAnalogX()>85)
    _rightFactor=abs(100/(float)cAnalogX());
  else{
    _analogZeroX = JOY_ZEROX;
    _analogZeroY = JOY_ZEROY;
  }
}

// Because gravity pulls down on the z-accelerometer while the nunchuk
// is upright, we need to calibrate {x,y} and {z} separately. Execute
// this function while the nunchuk is known to be upright and then 
// execute nunchuk_calibrate_accelz() when the nunchuk is on its side.
void ArduinoNunchuk::calibrate_accelxy(){
  _accelZeroX = accelX;
  _accelZeroY = accelY;
}

// See documentation for calibrate_accelxy()
void ArduinoNunchuk::calibrate_accelz(){
  _accelZeroZ = accelZ;
}

// Return calibrated x and y values of the joystick.
int ArduinoNunchuk::cAnalogX(){
  int temp = analogX - _analogZeroX;
  if (temp>0)
    return temp * _rightFactor > 100 ? 100 : temp * _rightFactor;
  else
    return temp * _leftFactor > 100 ? 100 : temp * _leftFactor;
}

int ArduinoNunchuk::cAnalogY(){
  int temp = analogY - _analogZeroY;
  if (temp>0)
    return temp * _upFactor > 100 ? 100 : temp * _upFactor;
  else
    return temp * _downFactor > 100 ? 100 : temp * _downFactor;
}

// Returns the x,y and z accelerometer values with calibration values
// subtracted.
int ArduinoNunchuk::cAccelX(){
    return accelX - _accelZeroX;
}

int ArduinoNunchuk::cAccelY(){
    return accelY - _accelZeroY;
}

int ArduinoNunchuk::cAccelZ(){
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
int ArduinoNunchuk::joyAngle(){
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
int ArduinoNunchuk::rollAngle(){
  return (int)(atan2((double)cAccelX(),(double)cAccelZ()) * RAD_TO_DEG);
}

// Returns pitch angle in degrees. Under the assumption that the
// only acceleration detected by the accelerometer is acceleration due
// to gravity, this function uses the ratio of the y and z
// accelerometer readings to gauge pitch.  This only works while the
// nunchuk is being held still or at constant velocity with zero ext-
// ernal force.
int ArduinoNunchuk::pitchAngle(){
  return (int)(atan2((double)cAccelY(),(double)cAccelZ()) * RAD_TO_DEG);
}

//Detect the states of the buttons and save them
void ArduinoNunchuk::_checkButtons()
{
  int buttons[] = {cButton,zButton};
  static bool prevState[NUM_BUTTONS];
  static bool currState[NUM_BUTTONS];
  int button;
  
  for (button = 0; button < NUM_BUTTONS; button++) {
    _justPressed[button] = false;
    _justReleased[button] = false;
     
    currState[button] = buttons[button]; // read the current state of the button
    /*
    Serial.print("Button Number ");
    Serial.print(button);
    Serial.print(" - Curr: ");
    Serial.print(currState[button]);
    Serial.print(", Prev:");
    Serial.print(prevState[button]);
    Serial.print(", Pressed:");
    */
    if (currState[button] != prevState[button]) {
      if (currState[button] == 1) {
        _justPressed[button] = true;
      }
      else if (currState[button] == 0) {
        _justReleased[button] = true;
      }
      _currentlyPressed[button] = currState[button];
    }
    prevState[button] = currState[button];
    /*
    Serial.print(_currentlyPressed[button]);
    Serial.print(", justPressed: ");
    Serial.print(_justPressed[button]);
    Serial.print(", justReleased: ");
    Serial.println(_justReleased[button]);
    */
  }
}

bool ArduinoNunchuk::justPressed(char button){
  switch(button){
    case 'c':
      return _justPressed[0];
    case 'z':
      return _justPressed[1];
  }
}

bool ArduinoNunchuk::justReleased(char button){
  switch(button){
    case 'c':
      return _justReleased[0];
    case 'z':
      return _justReleased[1];
  }
}

bool ArduinoNunchuk::currentlyPressed(char button){
  switch(button){
    case 'c':
      return _currentlyPressed[0];
    case 'z':
      return _currentlyPressed[1];
  }
}

// This function is custom made for myself, DONT USE IN YOUR PROGRAM
// These defaults are for my nunchuck ONLY
void ArduinoNunchuk::personalCalibrationValues(){
  _upFactor = (float)100/92;
  _downFactor = (float)100/103;
  _leftFactor = (float)100/96;
  _rightFactor = (float)100/95;
}
