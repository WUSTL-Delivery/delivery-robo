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
    
    // if (pwm > 0) {
    //     digitalWrite(DRIVE_MOTOR_IN1, HIGH);
    //     digitalWrite(DRIVE_MOTOR_IN2, LOW);
    //     analogWrite(DRIVE_MOTOR_PWM, constrain(pwm, 0, 255));

    //     Serial.print("mepoowParsed - Vel: ");
    //     Serial.print(constrain(pwm, 0, 255));
    //     Serial.print(" | Angle: ");
    //     Serial.println(arg2);
    // } else if (pwm < 0) {
    //     digitalWrite(DRIVE_MOTOR_IN1, LOW);
    //     digitalWrite(DRIVE_MOTOR_IN2, HIGH);
    //     analogWrite(DRIVE_MOTOR_PWM, constrain(-pwm*2, 0, 255));
    // } else {
    //     digitalWrite(DRIVE_MOTOR_IN1, LOW);
    //     digitalWrite(DRIVE_MOTOR_IN2, LOW);
    //     analogWrite(DRIVE_MOTOR_PWM, 0);
    // }

    // Cytron (24v) motor driver
    if (pwm < 0) {
        // Set motor to reverse and make pwm positive
        digitalWrite(DRIVE_MOTOR_DIR, LOW);
        pwm = -pwm;
    } else {
        digitalWrite(DRIVE_MOTOR_DIR, HIGH);
    }
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