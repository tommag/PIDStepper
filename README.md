This library is used to control a TMC5160 stepper motor driver by Trinamic, integrating a PID controller to smooth 
the motor movements while following a stream of target positions.
It can optionally use an encoder on the motor for the PID input.

This lib relies on the TMC5160 internal ramp generator. The acceleration is set at the beginning. 
When a new target position is received, it is sent to the TMC5160 ramp generator, and simultaneously fed to a PID
controller. This PID uses the current position as input (optionally provided by an encoder mounted on the motor) ; 
its output is the max allowed speed for the ramp generator. 

The base unit is the motor step.

Requirements : 
https://github.com/tommag/TMC5160_Arduino
https://github.com/br3ttb/Arduino-PID-Library
