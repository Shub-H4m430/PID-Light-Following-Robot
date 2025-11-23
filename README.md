Arduino Winter Internship 2025 - Task 1: Sensor-Actuator Integration

Project Title: PID-Controlled Light Following Robot

Submitted by: Shubham Dashrath Dombale 

Date: 23 November 2025

Submission Type: Task 1 (Sensor-Actuator Integration)

1. Project Overview
This project demonstrates the creation of an Autonomous Light Following Robot. While basic robots often use simple "If/Else" logic (which makes them jerk left and right), this robot uses a continuous control system called PID (Proportional-Integral-Derivative).

The robot mimics intelligent behavior:
Smooth Tracking: It curves smoothly towards a light source rather than zig-zagging.
Reactive Safety: If the light source gets too close, the robot detects the high intensity and reverses to maintain a safe distance.
Auto-Sleep: If the room becomes pitch black, the robot automatically stops to save power and prevent collisions.

2. Hardware Components
The robot is built using standard, accessible electronics components:
Brain: Arduino Uno (ATmega328P Microcontroller)
Eyes (Sensors): 3x Light Dependent Resistors (LDRs) arranged in a row (Left, Center, Right).
Muscles (Actuators): 2x DC Gear Motors (Yellow TT Motors) attached to wheels.
Nervous System (Driver): L298N Dual H-Bridge Motor Driver (Handles high current for motors).
Power: 2x 18650 Li-ion Batteries (Provides 7.4V - 8.4V).
Passive Components: 3x 10kΩ Resistors (Used to create voltage dividers for the sensors).

3. Circuit Logic & Wiring
The circuit connects the high-power components (Motors) and low-power components (Sensors) to the Arduino.

A. The Sensors (The "Pull-Down" Configuration)
The three LDR sensors are wired as Voltage Dividers. Based on the readings from the serial monitor, the sensor behavior is:
Bright Light: The sensor resistance drops, resulting in a Lower Analog Value (approx. 200-400).
Darkness: The sensor resistance increases, resulting in a Higher Analog Value (approx. 800-900).

Left Sensor: Connected to Pin A0
Center Sensor: Connected to Pin A1
Right Sensor: Connected to Pin A2

B. The Actuators (Motors)
The motors are controlled using PWM (Pulse Width Modulation), which allows us to control the speed of the motors, not just turn them On/Off.
Right Motor: Speed controlled by Pin 10. Direction controlled by Pins 4 and 7.
Left Motor: Speed controlled by Pin 11. Direction controlled by Pins 8 and 9.

4. The "Brain" of the Robot: PID Algorithm
The most complex part of this project is the math used to steer the robot. Instead of simple commands, we calculate an Error Value.

4.1. Calculating the Error
$$Error = \text{Right Sensor Value} - \text{Left Sensor Value}$$
Scenario 1 (Light is on the Left):
Left Sensor reads 200 (Bright). Right Sensor reads 900 (Dark).
$Error = 900 - 200 = \mathbf{+700}$.
Result: Positive error tells the robot to speed up the Right motor and slow down the Left motor $\rightarrow$ Turn Left.

Scenario 2 (Light is on the Right):
Left Sensor reads 900 (Dark). Right Sensor reads 200 (Bright).
$Error = 200 - 900 = \mathbf{-700}$.
Result: Negative error does the opposite $\rightarrow$ Turn Right.

4.2. The PID Formula
Once we have the Error, we process it through the PID formula to decide how much to turn.

PID\_Value = (K_p \times P) + (K_i \times I) + (K_d \times D)$$

P (Proportional): "How far is the light?"
If the error is huge (light is far to the side), P creates a strong turning force.
D (Derivative): "How fast are we turning?"
This acts like a shock absorber. If the robot swings towards the light too fast, the D-term detects the rapid change and counter-steers slightly to prevent the robot from "wobbling" or overshooting the target.
I (Integral): (Set to 0 for this project)
Accumulates error over time. Not needed for dynamic light tracking.

5. Intelligent Behaviors (State Machine)
The loop() function constantly checks the environment to decide which "State" the robot should be in:
REVERSE (Too Bright): Triggered when the Center Sensor reads below 500 (Very Bright). The robot interprets this as a collision risk and flips the motor direction to back up.
STOP (Too Dark): Triggered when both Left and Right sensors read above 800 (Darkness). The robot cuts power to prevent running away blindly.
TRACK (Normal): In normal lighting, the PID algorithm takes over, adjusting the left/right motor speeds continuously to keep the error close to zero (centered).

6. Calibration Details
Every motor is slightly different. During testing, I noticed the Left Motor was physically weaker than the Right Motor due to gearbox friction.
Correction: I set the BaseSpeed_Left to 110 and BaseSpeed_Right to 90.
Result: This software calibration forces the robot to drive perfectly straight when no steering is applied.

