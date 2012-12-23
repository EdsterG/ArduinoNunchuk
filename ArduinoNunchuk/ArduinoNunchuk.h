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

#ifndef ArduinoNunchuk_H
#define ArduinoNunchuk_H

#if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

class ArduinoNunchuk
{  
  public:  
    int analogX;
    int analogY;
    int accelX;
    int accelY;
    int accelZ;
    int zButton;
    int cButton;
  
    void init();  
    void update();
    int cAnalogX();
    int cAnalogY();
    int cAccelX();
    int cAccelY();
    int cAccelZ();
    int joyAngle();
    int rollAngle();
    int pitchAngle();
    void calibrate_joy();
    void calibrate_accelxy();
    void calibrate_accelz();
    static void wiiChuckPwr(); 
    
  private: 
    int _analogZeroX;
    int _analogZeroY;
    int _accelZeroX;
    int _accelZeroY;
    int _accelZeroZ; 
    void _sendByte(byte data, byte location);
};

#endif
