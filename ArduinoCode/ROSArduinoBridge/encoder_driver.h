/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
   
#ifdef ARDUINO_ENC_COUNTER
  //below can be changed, but should be PORTD pins; 
  //otherwise additional changes in the code are required
  #define LEFT_ENC_PIN_A 39
  #define LEFT_ENC_PIN_B 34
  
  //below can be changed, but should be PORTC pins
  #define RIGHT_ENC_PIN_A 35
  #define RIGHT_ENC_PIN_B 32
#endif


// прерывания для esp32
#ifdef ARDUINO_ENC_COUNTER
void IRAM_ATTR leftEncoderISR();
void IRAM_ATTR rightEncoderISR();
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

