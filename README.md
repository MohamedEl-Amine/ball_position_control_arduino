# Ball Position Control System with Arduino and Ultrasonic Sensors

This project aims to create a system that can control the position of a ball inside a tube using an Arduino microcontroller and ultrasonic sensors. The system detects the position of a hand near the bottom of the tube and activates a motor to move the ball to a specific position relative to a sensor located at the top of the tube. This project demonstrates the use of sensors, PID (Proportional-Integral-Derivative) control, and motor control with Arduino.

## Components:

- **Arduino Uno**: The main microcontroller board that controls the entire system.
- **HC-SR04 Ultrasonic Sensors**: Two sensors are used—one positioned at the bottom of the tube to detect the hand's distance and another at the top to detect the ball's distance.
- **DC Motor**: A motor is used to move the ball within the tube.
- **Transistor and Resistor**: Used to control the motor's speed and direction.
- **Power Supply**: Provides power to the Arduino and motor.

## Pin Configuration:

- **Bottom Sensor**:
  - Trigger Pin: 3
  - Echo Pin: 4
- **Top Sensor**:
  - Trigger Pin: 10
  - Echo Pin: 11
- **Motor**: Defined on Pin 9.

## Operation:

1. The system first detects the hand position using the bottom sensor. If the hand is within a certain range (e.g., between 11 and 31 centimeters from the bottom sensor), it proceeds to the next step.
2. The distance from the bottom sensor to the hand position is calculated, and based on this, the desired position for the ball relative to the top sensor is determined.
3. The system then measures the actual distance of the ball from the top sensor and calculates the error.
4. Using a PID controller, the system determines the appropriate speed and direction to move the motor in order to position the ball correctly relative to the top sensor.

## How to Use:

1. Connect the components according to the specified pin configuration.
2. Upload the Arduino code provided in this repository to your Arduino Uno.
3. Power on the system and place your hand near the bottom sensor to activate the control system.
4. Observe the motor adjusting the ball position based on hand movement.
