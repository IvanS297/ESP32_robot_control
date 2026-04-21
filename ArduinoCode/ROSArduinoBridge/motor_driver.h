/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 33
  #define LEFT_MOTOR_BACKWARD  26
  #define RIGHT_MOTOR_FORWARD  25
  #define LEFT_MOTOR_FORWARD   27
  //#define RIGHT_MOTOR_ENABLE 12
  //#define LEFT_MOTOR_ENABLE 13
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
