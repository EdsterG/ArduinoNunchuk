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

#define NUM_BUTTONS 2

class ArduinoNunchuk
{  
  public:
    void init(bool wiichuck = true);
    int analogX;
    int analogY;
    int accelX;
    int accelY;
    int accelZ;
    int zButton;
    int cButton;
  
    void update();
    int cAnalogX();
    int cAnalogY();
    int cAccelX();
    int cAccelY();
    int cAccelZ();
    int joyAngle();
    int rollAngle();
    int pitchAngle();
    void calibrate_joy(char location[] = "center");
    void calibrate_accelxy();
    void calibrate_accelz();
    bool justPressed(char button);
    bool justReleased(char button);
    bool currentlyPressed(char button);

    // This function is custom made for myself, DONT USE IN YOUR PROGRAM
    void personalCalibrationValues();

  private:

    bool _currentlyPressed[NUM_BUTTONS];
    bool _justPressed[NUM_BUTTONS];
    bool _justReleased[NUM_BUTTONS];
    void _checkButtons();
    int _analogZeroX;
    int _analogZeroY;
    int _accelZeroX;
    int _accelZeroY;
    int _accelZeroZ;
    float _upFactor;
    float _downFactor;
    float _leftFactor;
    float _rightFactor;

    static void _wiiChuckPwr(); 
    static void _sendByte(byte data, byte location);
};

#endif
