/* *************************************************************
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */
  
#ifdef USE_BASE

#ifdef ROBOGAIA
  /* Wrap the encoder reset function */
  long readEncoder(int i) {
    return 0;
  }
  void resetEncoder(int i) {}
  void resetEncoders() {}

// переделано под esp32
#elif defined(ARDUINO_ENC_COUNTER)
  // ESP32 версия энкодеров через прерывания
  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;

  // Пины энкодеров (можно настроить в encoder_driver.h)
  #define LEFT_ENC_PIN_A 39
  #define LEFT_ENC_PIN_B 34
  #define RIGHT_ENC_PIN_A 35
  #define RIGHT_ENC_PIN_B 32

  // Обработчики прерываний для левого энкодера
  void IRAM_ATTR leftEncoderISR() {
    static uint8_t lastState = 0;
    uint8_t state = (digitalRead(LEFT_ENC_PIN_A) << 1) | digitalRead(LEFT_ENC_PIN_B);
    if (state != lastState) {
      if ((lastState == 0 && state == 1) || (lastState == 1 && state == 3) ||
          (lastState == 3 && state == 2) || (lastState == 2 && state == 0))
        left_enc_pos++;
      else
        left_enc_pos--;
      lastState = state;
    }
  }

  void IRAM_ATTR rightEncoderISR() {
    static uint8_t lastState = 0;
    uint8_t state = (digitalRead(RIGHT_ENC_PIN_A) << 1) | digitalRead(RIGHT_ENC_PIN_B);
    if (state != lastState) {
      if ((lastState == 0 && state == 1) || (lastState == 1 && state == 3) ||
          (lastState == 3 && state == 2) || (lastState == 2 && state == 0))
        right_enc_pos++;
      else
        right_enc_pos--;
      lastState = state;
    }
  }

  long readEncoder(int i) {
    if (i == LEFT) return left_enc_pos;
    else return right_enc_pos;
  }

  void resetEncoder(int i) {
    if (i == LEFT) left_enc_pos = 0L;
    else right_enc_pos = 0L;
  }

  void resetEncoders() {
    resetEncoder(LEFT);
    resetEncoder(RIGHT);
  }
#else
  #error A encoder driver must be selected!
#endif

#endif // USE_BASE