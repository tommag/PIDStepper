/* This code is derived from TMC5160 SPI example

Hardware setup :
Connect the following lines between the microcontroller board and the TMC5160 driver
(Tested with a Teensy 3.2 and a TMC5160-BOB)

  MOSI (Teensy : 11)  <=> SDI
  MISO (Teensy : 12)  <=> SDO
  SCK (Teensy : 13)   <=> SCK
  5                   <=> CSN
  8                   <=> DRV_ENN (optional, tie to GND if not used)
  GND                 <=> GND
  3.3V/5V             <=> VCC_IO (depending on the processor voltage)

The TMC5160 VS pin must also be powered.
Tie CLK16 to GND to use the TMC5160 internal clock.
Tie SPI_MODE to VCC_IO, SD_MODE to GND.

Connect the encoder to the TMC5160 ENCA, ENCB inputs

Please run the Config Wizard for power stage fine tuning / motor calibration (this code uses the default parameters and auto calibration).

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

#include <Arduino.h>
#include <TMC5160.h>
#include <PIDStepper.h>

const uint8_t SPI_CS = 5; // CS pin in SPI mode
const uint8_t SPI_DRV_ENN = 8;  // DRV_ENN pin in SPI mode

TMC5160_SPI motor = TMC5160_SPI(SPI_CS); //Use default SPI peripheral and SPI settings.
PIDStepper pidController = PIDStepper(motor, 10, 1, 0, 100, true);


void setup()
{
  // USB/debug serial coms
  Serial.begin(115200);

  pinMode(SPI_DRV_ENN, OUTPUT); 
  digitalWrite(SPI_DRV_ENN, LOW); // Active low

  // This sets the motor & driver parameters /!\ run the configWizard for your driver and motor for fine tuning !
  TMC5160::PowerStageParameters powerStageParams; // defaults.
  TMC5160::MotorParameters motorParams;
  motorParams.globalScaler = 98; // Adapt to your driver and motor (check TMC5160 datasheet - "Selecting sense resistors")
  motorParams.irun = 31;
  motorParams.ihold = 16;

  SPI.begin();
  motor.begin(powerStageParams, motorParams, TMC5160::NORMAL_MOTOR_DIRECTION);
  motor.setEncoderResolution(200, 4000, false);
  motor.setEncoderAllowedDeviation(1);

  // ramp definition
  motor.setRampMode(TMC5160::POSITIONING_MODE);
  motor.setAcceleration(2000);
  pidController.setMaxSpeed(1000); // Set max speed in the PID controller

  Serial.println("starting up");

  delay(1000); // Standstill for automatic tuning
}

void loop()
{
  pidController.run(); // Call in main loop()

  uint32_t now = millis();
  static unsigned long t_dirchange, t_echo;
  static bool dir;

  // every n seconds or so...
  if ( now - t_dirchange > 3000 )
  {
    t_dirchange = now;

    // reverse direction
    dir = !dir;
    pidController.setTargetPosition(dir ? 200 : 0);  // Set target position in the PID controller
  }

  if( now - t_echo > 100 )
  {
    t_echo = now;

    Serial.print("Target pos: ");
    Serial.println(motor.getTargetPosition());
    Serial.print("Current pos: ");
    Serial.println(motor.getCurrentPosition());
    Serial.print("PID output: ");
    Serial.println(pidController.getSpeed());
  }
}
