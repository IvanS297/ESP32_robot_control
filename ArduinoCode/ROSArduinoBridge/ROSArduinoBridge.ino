#define USE_BASE  // Enable the base controller code
//#undef USE_BASE     // Disable the base controller code

/* Define the motor controller and encoder library you are using */
#ifdef USE_BASE
/* The Pololu VNH5019 dual motor driver shield */
//#define POLOLU_VNH5019

/* The Pololu MC33926 dual motor driver shield */
//#define POLOLU_MC33926

/* The RoboGaia encoder shield */
//#define ROBOGAIA

/* Encoders directly attached to Arduino board */
//#define ARDUINO_ENC_COUNTER
#define ROBOGAIA

/* L298 Motor driver*/
#define L298_MOTOR_DRIVER
#endif

//#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h
#undef USE_SERVOS  // Disable use of PWM servos

/* Serial port baud rate */
#define BAUDRATE 115200

/* Maximum PWM signal */
#define MAX_PWM 255

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Include definition of serial commands */
#include "commands.h"

/* Sensor functions */
#include "sensors.h"

/* Include servo support if required */
#ifdef USE_SERVOS
#include <Servo.h>
#include "servos.h"
#endif

#ifdef USE_BASE
/* Motor driver function definitions */
#include "motor_driver.h"

/* Encoder driver function definitions */
#include "encoder_driver.h"

/* PID parameters and functions */
#include "diff_controller.h"

/* Run the PID loop at 30 times per second */
#define PID_RATE 30  // Hz

/* Convert the rate into an interval */
const int PID_INTERVAL = 1000 / PID_RATE;

/* Track the next time we make a PID calculation */
unsigned long nextPID = PID_INTERVAL;

/* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
#define AUTO_STOP_INTERVAL 2000
long lastMotorCommand = AUTO_STOP_INTERVAL;
#endif

/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int idx = 0;  // assignment of function 'char* index(const char*, int)'

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

/* Clear the current command parameters */
void resetCommand() {
  cmd = '\0';  // было NULL
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  idx = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch (cmd) {
    case GET_BAUDRATE:
      Serial.println(BAUDRATE);
      break;
    case ANALOG_READ:
      Serial.println(analogRead(arg1));
      break;
    case DIGITAL_READ:
      Serial.println(digitalRead(arg1));
      break;
    case ANALOG_WRITE:
      analogWrite(arg1, arg2);
      Serial.println("OK");
      break;
    case DIGITAL_WRITE:
      if (arg2 == 0) digitalWrite(arg1, LOW);
      else if (arg2 == 1) digitalWrite(arg1, HIGH);
      Serial.println("OK");
      break;
    case PIN_MODE:
      if (arg2 == 0) pinMode(arg1, INPUT);
      else if (arg2 == 1) pinMode(arg1, OUTPUT);
      Serial.println("OK");
      break;
    case PING:
      Serial.println(Ping(arg1));
      break;
#ifdef USE_SERVOS
    case SERVO_WRITE:
      servos[arg1].setTargetPosition(arg2);
      Serial.println("OK");
      break;
    case SERVO_READ:
      Serial.println(servos[arg1].getServo().read());
      break;
#endif

#ifdef USE_BASE
    case READ_ENCODERS:
      Serial.print(readEncoder(LEFT));
      Serial.print(" ");
      Serial.println(readEncoder(RIGHT));
      break;
    case RESET_ENCODERS:
      resetEncoders();
      resetPID();
      Serial.println("OK");
      break;
    case MOTOR_SPEEDS:
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      if (arg1 == 0 && arg2 == 0) {
        setMotorSpeeds(0, 0);
        resetPID();
        moving = 0;
      } else moving = 1;
      leftPID.TargetTicksPerFrame = arg1;
      rightPID.TargetTicksPerFrame = arg2;
      Serial.println("OK");
      break;
    case MOTOR_RAW_PWM:
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      resetPID();
      moving = 0;  // Sneaky way to temporarily disable the PID
      setMotorSpeeds(arg1, arg2);
      Serial.println("OK");
      break;
    case UPDATE_PID:
      while ((str = strtok_r(p, ":", &p)) != NULL) {  // исправление за автором: strtok возвращает указатель а не символ
        pid_args[i] = atoi(str);
        i++;
      }
      Kp = pid_args[0];
      Kd = pid_args[1];
      Ki = pid_args[2];
      Ko = pid_args[3];
      Serial.println("OK");
      break;
#endif
    default:
      Serial.println("Invalid Command");
      break;
  }
  // while (Serial.available()) {
  //   char c = Serial.peek();
  //   if (c == '\n' || c == '\r') {
  //     Serial.read();  // съедаем \n или \r
  //     break;
  //   }
  //   Serial.read();  // иначе пропускаем
  // }
}

/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);

// Initialize the motor controller if used */
#ifdef USE_BASE
#ifdef ARDUINO_ENC_COUNTER
  pinMode(LEFT_ENC_PIN_A, INPUT);
  pinMode(LEFT_ENC_PIN_B, INPUT);
  pinMode(RIGHT_ENC_PIN_A, INPUT);
  pinMode(RIGHT_ENC_PIN_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_B), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_A), rightEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_B), rightEncoderISR, CHANGE);
#endif
  initMotorController();  // без EN пинов
  resetPID();
#endif

  /* Attach servos if used */
#ifdef USE_SERVOS
  int i;
  for (i = 0; i < N_SERVOS; i++) {
    servos[i].initServo(
      servoPins[i],
      stepDelay[i],
      servoInitPosition[i]);
  }
#endif
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {
  while (Serial.available() > 0) {
    chr = Serial.read();

    if (chr == '\r' || chr == '\n') {
      if (arg == 1) argv1[idx] = '\0';
      else if (arg == 2) argv2[idx] = '\0';
      if (cmd != '\0') {
        runCommand();
        resetCommand();
      }
      // После обработки команды очищаем остаток буфера до конца строки
      while (Serial.available()) {
        char c = Serial.peek();
        if (c == '\r' || c == '\n') {
          Serial.read();
        } else {
          break;
        }
      }
      continue;
    }

    if (chr == ' ') {
      if (arg == 0) arg = 1;
      else if (arg == 1) {
        argv1[idx] = '\0';
        arg = 2;
        idx = 0;
      }
      continue;
    }

    if (arg == 0) {
      cmd = chr;
    } else if (arg == 1) {
      if (idx < 15) argv1[idx++] = chr;
    } else if (arg == 2) {
      if (idx < 15) argv2[idx++] = chr;
    }
  }


// If we are using base control, run a PID calculation at the appropriate intervals
#ifdef USE_BASE
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }

  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    ;
    setMotorSpeeds(0, 0);
    moving = 0;
  }
#endif

// Sweep servos
#ifdef USE_SERVOS
  int i;
  for (i = 0; i < N_SERVOS; i++) {
    servos[i].doSweep();
  }
#endif
}
