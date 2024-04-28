#include "Ultrasonic.h" // Include the Ultrasonic library

// Define pin numbers
const int bottomTrigPin = 3; // Trigger pin for bottom HC-SR0 sensor
const int bottomEchoPin = 4; // Echo pin for bottom HC-SR0 sensor
const int topTrigPin = 10; // Trigger pin for top HC-SR0 sensor
const int topEchoPin = 11; // Echo pin for top HC-SR0 sensor
const int motorPin = 9; // Pin for controlling the motor

// Create Ultrasonic objects for sensors
Ultrasonic bottomSensor(bottomTrigPin, bottomEchoPin); // Initialize bottom sensor
Ultrasonic topSensor(topTrigPin, topEchoPin); // Initialize top sensor

// Define constants for PID
const double Kp = 0.05; // Proportional constant (Adjust as needed)
const double Ki = 0.005;     // Integral constant (Adjust as needed)
const double Kd = 0.1; // Derivative constant (Adjust as needed)
double prevError = 0;
double integral = 0;
double integralLimit = 100; // Limit for integral term accumulation
double outputLimit = 255;    // Limit for PID controller output

// Define variables for smooth deceleration
bool decelerating = false;
double decelerationRate = 0.02; // Adjust as needed
double minSpeed = 50; // Minimum speed threshold

// PID function to compute motor speed based on error
int pidController(double error) {
    // Calculate proportional term
    double proportional = Kp * error;

    // Calculate integral term
    integral += error * Ki;
    integral = constrain(integral, -integralLimit, integralLimit); // Limit integral term accumulation

    // Calculate derivative term
    double derivative = Kd * (error - prevError);

    // Calculate PID output
    double output = proportional + integral + derivative;

    // Constrain PID output within limits
    output = constrain(output, -outputLimit, outputLimit);

    // Update previous error
    prevError = error;
    Serial.print("output : ");
    Serial.println(output);
    // Adjust motor speed based on PID output
    int motorSpeed;
    if (output >= 0) {
        motorSpeed = constrain(int(output), 0, 255);
    } else {
        motorSpeed = 0;
    }

    // Check if decelerating
    if (decelerating && motorSpeed > minSpeed) {
        // Gradually decrease the motor speed
        motorSpeed -= decelerationRate;
        // Ensure motor speed does not go below the minimum threshold
        motorSpeed = max(motorSpeed, minSpeed);
    }

    return motorSpeed;
}

void setup() {
    pinMode(motorPin, OUTPUT);

    // Initialize Serial communication
    Serial.begin(9600);
}

// Ultrasonic function
double getDistance(Ultrasonic sensor) {
    return sensor.read(CM);
}

void loop() {
    // Read distance from bottom detector
    double bottomDetectionHandDistance = getDistance(bottomSensor);

    // If hand is detected within the desired range, start the motor
    if (bottomDetectionHandDistance <= 31 && bottomDetectionHandDistance >= 11) {
        // Read distance of the ball from top detector
        double ballDistanceFromTopDetector = getDistance(topSensor);

        // Calculate Setpoint for PID
        double dist_reel_hand = bottomDetectionHandDistance - 11;

        // Calculate Setpoint for PID
        double Setpoint = 20 - dist_reel_hand + 7;

        Serial.print("Setpoint : ");
        Serial.println(Setpoint);

        // Calculate error between Setpoint and actual distance
        double error = ballDistanceFromTopDetector - Setpoint ;

        // Compute motor speed using PID function
        int motorSpeed = pidController(error);

        // Adjust motor speed based on PID output
        analogWrite(motorPin, motorSpeed);

        // Check if decelerating
        if (error < 1.0) {
            decelerating = true;
        }

        // Print debug information
        Serial.print("Error: ");
        Serial.print(error);
        Serial.print(" | PID Output: ");
        Serial.print(motorSpeed);
        Serial.print(" | Actual Distance: ");
        Serial.println(ballDistanceFromTopDetector);
    } else {
        // If hand is not detected, stop the motor
        analogWrite(motorPin, 0);
        decelerating = false; // Reset deceleration flag
    }

    delay(5); // Adjust delay as needed for desired loop frequency
}
