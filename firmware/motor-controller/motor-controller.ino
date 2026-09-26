#include "commands.h"
#include "motor_driver.h"
#include "battery.h"

#define BAUDRATE 57600

// Command parsing variables
int arg = 0;
int index = 0;
char chr;
char cmd;
char argv1[ARG_MAX_LEN];
char argv2[ARG_MAX_LEN];
double arg1;
double arg2;

void runCommand();
void resetCommand();
void errorCommand();
void processBuffer();

void setup() {
  Serial.begin(BAUDRATE);
  // pinMode(LED_BUILTIN, OUTPUT);
  motorInit(); // initialize motors
  encoderInit(); //initialize encoder
  batteryInit();
}

void loop() {
  processBuffer();
  batteryUpdate();
  // motorUpdate(); // keep PID running
}

void runCommand() {
  switch (cmd) {
    case SET_DRIVE_PWM: // "m val1 val2"
      setDriveSpeed((int)arg1);
      setTurnAngle((int)arg2);
      Serial.print("Parsed - Vel: ");
      Serial.print((int)arg1);
      Serial.print(" | Angle: ");
      Serial.println((int)arg2);

      break;

    // case SET_DRIVE_VEL: // "v val"
    //   setDriveVel(arg1);
    //   break;

    // case SET_TURN_ANGLE: // "r val"
    //   setTurnAngle(arg1);
    //   break;

    case SET_PID: // "u kp:kd:ki:ko"
      // TODO: parse colon-delimited values and call setPidValues()
      break;

    case GET_ENCODER: // "e"
      Serial.println(getEncoder());
      break;
    
    case GET_BATTERY: // "b"
      Serial.print(readVoltage());
      break;

    case TEST_BLINK_ON: // "o"
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.println("test on");
      break;

    case TEST_BLINK_OFF: // "f"
      digitalWrite(LED_BUILTIN, LOW);
      Serial.println("test off");
      break;

    default:
      errorCommand();
  }
}

void resetCommand() {
  arg = 0;
  index = 0;
  cmd = '\0';
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
}

void errorCommand() {
  Serial.print(ERROR_RES);
  Serial.print(NEW_COMMAND);
  resetCommand();
}

void processBuffer() {
  while (Serial.available() > 0) {
    chr = Serial.read();

    // Ignore newline (\n) if present
    if (chr == '\n') continue;

    if (chr == NEW_COMMAND) {
      if (arg == 1) argv1[index] = '\0';
      else if (arg == 2) argv2[index] = '\0';

      arg1 = atof(argv1);
      arg2 = atof(argv2);

      // No debug echo here: at 20 Hz commands + 20 Hz encoder polls it used ~45% of the
      // 57600-baud link and buried the reply to 'e' (see hardware_bringup/serial_protocol.py).
      if (cmd != '\0') runCommand();
      resetCommand();
    }
    else if (chr == NEW_ARG) {
      if (arg == 0) arg++;
      else if (arg == 1) {
        argv1[index] = '\0';
        arg++;
        index = 0;
      }
    }
    else {
      if (arg == 0) {
        if (cmd != '\0') errorCommand();
        else cmd = chr;
      } else if (arg == 1) {
        argv1[index++] = chr;
      } else if (arg == 2) {
        argv2[index++] = chr;
      }

      if (index == ARG_MAX_LEN) errorCommand();
    }
  }
}
