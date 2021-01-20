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

#include "PIDStepper.h"

PIDStepper::PIDStepper(TMC5160& motor, double Kp, double Ki, double Kd, bool useEncoder) :
  _motor(&motor),
  _useEncoder(useEncoder),
  _pid(&_pidInput, &_pidOutput, &_pidSetpoint, Kp, Ki, Kd, P_ON_E, DIRECT)
{
  _pid.SetMode(AUTOMATIC);
  _pid.SetSampleTime(1); // Compute at 1000Hz
}

void PIDStepper::run() 
{
  if (_useEncoder && _motor->isEncoderDeviationDetected())
  {
    _motor->setCurrentPosition(_motor->getEncoderPosition(), false); // Correct internal actual position with encoder position (/!\ may glitch at high speed ?)
    _motor->clearEncoderDeviationFlag();
  }

  _pidInput = _motor->getCurrentPosition();

  if (_pid.Compute())
    _motor->setMaxSpeed(_pidOutput);
}

void PIDStepper::setMaxSpeed(float maxSpeed_steps_s)
{
  _pid.SetOutputLimits(-abs(maxSpeed_steps_s), abs(maxSpeed_steps_s));
}

void PIDStepper::setTargetPosition(float targetPos_steps)
{
  _motor->setTargetPosition(targetPos_steps);
  _pidSetpoint = targetPos_steps;
}

void PIDStepper::setGains(double Kp, double Ki, double Kd)
{
  _pid.SetTunings(Kp, Ki, Kd);
}

float PIDStepper::getSpeed()
{
  return (float)_pidOutput;
}