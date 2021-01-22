/*
MIT License

Copyright (c) 2021 Tom Magnier

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef PID_STEPPER_H
#define PID_STEPPER_H

#include <Arduino.h>
#include <TMC5160.h>
#include <PID_v1.h>

class PIDStepper {
public:
  /* Initialize library with PID gains */
  PIDStepper(TMC5160& motor, double Kp, double Ki, double Kd, unsigned int updateRate_Hz, bool useEncoder = false);

  /* Call this function as frequently as possible, preferably in the main loop */
  void run();

  /* Set the max allowed speed in steps/second. This is used to clamp the PID output. */
  void setMaxSpeed(float maxSpeed_steps_s);

  /* Set the target position in steps. This is the PID setpoint. */
  void setTargetPosition(float targetPos_steps);

  /* Update the PID gains */
  void setGains(double Kp, double Ki, double Kd);

  /* Return the current max speed (PID output) */
  float getSpeed();

private:
  bool _useEncoder;
  TMC5160 *_motor;
  double _pidSetpoint, _pidInput, _pidOutput;
  PID _pid;
};



#endif //PID_STEPPER_H