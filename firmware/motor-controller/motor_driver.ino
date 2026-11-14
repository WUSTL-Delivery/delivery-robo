// #include "PID_v1.h"
#include "motor_driver.h"
#include "encoder_driver.h"
#include <Servo.h>  // Add this!

Servo turnServo;  // Create servo object

double vel_input, vel_output, vel_target;
double turn_input, turn_output, turn_target;

// PID vel_control(&vel_input, &vel_output, &vel_target, KP_V, KI_V, KD_V, DIRECT);
// PID turn_control(&turn_input, &turn_output, &turn_target, KP_T, KI_T, KD_T, DIRECT);

void setDriveSpeed(int pwm) {
    // 1. Determine the Direction (DIR Pin)
    if (pwm < 0) {
        // Reverse
        digitalWrite(DRIVE_MOTOR_DIR, LOW);
        // Motor speed is the absolute value of pwm
        pwm = -pwm;
    } else {
        // Forward (or stopped, but we set the forward direction for when PWM > 0)
        digitalWrite(DRIVE_MOTOR_DIR, HIGH);
    }
    
    // 2. Set the Speed (PWM Pin)
    // constrain(pwm, 0, 255) ensures speed is between 0 and 255.
    // If pwm was 0 initially, this still works fine.
    analogWrite(DRIVE_MOTOR_PWM, constrain(pwm, 0, 255));
}
void setTurnAngle(int angle) {
    turnServo.write(constrain(angle, 0, 180));
}

void motorInit() {
    pinMode(DRIVE_MOTOR_PWM, OUTPUT);
    pinMode(DRIVE_MOTOR_DIR, OUTPUT);
    // pinMode(DRIVE_MOTOR_IN1, OUTPUT);
    // pinMode(DRIVE_MOTOR_IN2, OUTPUT);
    // digitalWrite(DRIVE_MOTOR_PWM, HIGH);
    digitalWrite(TURN_SERVO, LOW);  // Ensure LOW before attach
    delay(100);
    turnServo.attach(TURN_SERVO);
    // turnServo.write(90); 
    // pinMode(TURN_MOTOR_PWM, OUTPUT);
    // pinMode(TURN_MOTOR_IN1, OUTPUT);
    // pinMode(TURN_MOTOR_IN2, OUTPUT);
    //Serial.println("REAEDY!");
}

// void motorUpdate() {
//     vel_input = getDriveVel();
//     vel_control.Compute();
//     setMotorPWM(DRIVE_MOTOR, vel_output);

//     turn_input = getTurnAngle();
//     turn_control.Compute();
//     setMotorPWM(TURN_MOTOR, turn_output);
// }
