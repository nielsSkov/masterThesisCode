/*
  Joint.h - Library for reading input and setting output of joint on pendulum.
  Created by Rasmus Pedersen, July 8, 2014.
*/
#ifndef Joint_teensy_h
#define Joint_teensy_h

#include "Arduino.h"

#define DAC0_pin A21
#define DAC1_pin A22

class Joint
{
  public:
    Joint(int joint, float Ts);   // 1 = sled, 2 = pendulum 1, 3 = pendulum 2.
    void init();
    float readPos();
	float readVel();
    void resetPos();
    void setOutput(float value,bool current_input);
    int safetyStop();
  private:
	float _Ts;
	float _oldPos;
    int _pinOE;
    int _pinSEL;
    int _pinRST;
    int _dataBusStartPin;
    int _outputSEL;
    byte readByte(int startPin);
};

#endif