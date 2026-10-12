/**
 * @file swarm_agent.cpp
 * @brief micro-ROS Agent for ESP32 Mobile Robot
 * 
 * Author  : Namrata Kumari
 * Guide   : Prof. Dr. Sohom Chakrabarty
 * Institute: Indian Institute of Technology Roorkee
 * Year    : 2026
 */

#include <Arduino.h>
#include "swarm_controller.hpp"

// ================== PIN DEFINITIONS ==================
// Change these pins according to your actual wiring
#define MOTOR_LEFT_PWM    25
#define MOTOR_LEFT_IN1    26
#define MOTOR_LEFT_IN2    27

#define MOTOR_RIGHT_PWM   14
#define MOTOR_RIGHT_IN1   12
#define MOTOR_RIGHT_IN2   13

#define TRIG_PIN          5
#define ECHO_PIN          18

// ================== AGENT ID ==================
// Change this number for each robot (0, 1, 2, 3...)
#define MY_AGENT_ID       0

// Create controller object
SwarmController controller(MY_AGENT_ID);

// ================== ULTRASONIC FUNCTION ==================
float readObstacleDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 25000);  // timeout 25ms
    float distance = duration * 0.0001715f;          // convert to meters

    if (distance < 0.02f || distance > 2.5f) {
        return 2.5f;   // no valid obstacle
    }
    return distance;
}

// ================== MOTOR CONTROL ==================
void setMotorSpeeds(float linear, float angular) {
    // Simple differential drive
    float left_speed  = linear - (angular * 0.15f);  // 0.15 = half wheelbase
    float right_speed = linear + (angular * 0.15f);

    // Convert to PWM (0-255)
    int left_pwm  = constrain(abs(left_speed)  * 800, 0, 255);
    int right_pwm = constrain(abs(right_speed) * 800, 0, 255);

    // Left motor direction
    if (left_speed >= 0) {
        digitalWrite(MOTOR_LEFT_IN1, HIGH);
        digitalWrite(MOTOR_LEFT_IN2, LOW);
    } else {
        digitalWrite(MOTOR_LEFT_IN1, LOW);
        digitalWrite(MOTOR_LEFT_IN2, HIGH);
    }

    // Right motor direction
    if (right_speed >= 0) {
        digitalWrite(MOTOR_RIGHT_IN1, HIGH);
        digitalWrite(MOTOR_RIGHT_IN2, LOW);
    } else {
        digitalWrite(MOTOR_RIGHT_IN1, LOW);
        digitalWrite(MOTOR_RIGHT_IN2, HIGH);
    }

    analogWrite(MOTOR_LEFT_PWM, left_pwm);
    analogWrite(MOTOR_RIGHT_PWM, right_pwm);
}

// ================== SETUP ==================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("========================================");
    Serial.println("  Swarm Agent Starting");
    Serial.println("  Author : Namrata Kumari");
    Serial.println("  Guide  : Prof. Dr. Sohom Chakrabarty");
    Serial.println("========================================");

    // Motor pins
    pinMode(MOTOR_LEFT_PWM, OUTPUT);
    pinMode(MOTOR_LEFT_IN1, OUTPUT);
    pinMode(MOTOR_LEFT_IN2, OUTPUT);
    pinMode(MOTOR_RIGHT_PWM, OUTPUT);
    pinMode(MOTOR_RIGHT_IN1, OUTPUT);
    pinMode(MOTOR_RIGHT_IN2, OUTPUT);

    // Ultrasonic pins
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    // Stop motors at start
    analogWrite(MOTOR_LEFT_PWM, 0);
    analogWrite(MOTOR_RIGHT_PWM, 0);

    Serial.printf("Agent ID %d is ready\n", MY_AGENT_ID);
}

// ================== MAIN LOOP ==================
void loop() {
    // Read obstacle distance
    float obs_dist = readObstacleDistance();

    // Update the swarm controller (Bloch + velocity algorithm)
    controller.update(obs_dist);

    // Get results from controller
    float heading = controller.getHeading();
    float speed   = controller.getSpeed();
    float s       = controller.getSlowState();

    // Temporary: convert heading error to angular velocity
    float angular = 0.0f;   // will be improved later

    // Send commands to motors
    setMotorSpeeds(speed, angular);

    // Debug print
    Serial.printf("ID:%d | s:%.2f | speed:%.2f | heading:%.1f | obs:%.2f\n",
                  MY_AGENT_ID, s, speed, heading * 180.0 / PI, obs_dist);

    delay(50);  // 20 Hz control loop
}
