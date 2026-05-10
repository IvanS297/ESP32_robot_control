// ========== НАСТРОЙКИ ==========
#define USE_BASE               // Используем базовое управление (моторы + энкодеры)
#define USE_PID             // Раскомментируйте, если нужен PID (требует энкодеры)
//#define USE_WATCH_DOG       // Если хотите останавливать моторы через AUTO_STOP_INTERVAL при отсутствии команд

#define BAUDRATE 115200
#define MAX_PWM 255

// Пины моторов L298 (ШИМ-совместимые на ESP32)
#define LEFT_MOTOR_FORWARD   26 //33
#define LEFT_MOTOR_BACKWARD  27 //25
#define RIGHT_MOTOR_FORWARD  33 //26
#define RIGHT_MOTOR_BACKWARD 25 //27

// Пины энкодеров (обязательно с INPUT_PULLUP или внешними резисторами)
#define LEFT_ENC_A 39  
#define LEFT_ENC_B 34
#define RIGHT_ENC_A 32
#define RIGHT_ENC_B 35

// Команды (как в оригинале)
#define READ_ENCODERS  'e'
#define RESET_ENCODERS 'r'
#define MOTOR_SPEEDS   'm'
#define MOTOR_RAW_PWM  'o'
#define UPDATE_PID     'u'
#define LEFT           0  
#define RIGHT          1

// ========== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ==========
volatile long left_ticks = 0;
volatile long right_ticks = 0;

unsigned long nextPID = 0;
const int PID_INTERVAL = 33; // ~30 Hz
unsigned long lastMotorCommand = 0;
#ifdef USE_WATCH_DOG
  const unsigned long AUTO_STOP_INTERVAL = 1000;
#endif

bool moving = false;

// Переменные для парсинга команд
int arg = 0;
int idx = 0;
char chr;
char cmd = '\0';
char argv1[16];
char argv2[16];
long arg1 = 0, arg2 = 0;

#ifdef USE_PID
  // PID-структура
  struct SetPointInfo {
    double target;   // целевая скорость в тиках/интервал
    long prev_enc;
    int prev_input;
    int iterm;
    long output;
  };
  SetPointInfo leftPID, rightPID;
  int Kp = 10, Kd = 12, Ki = 0, Ko = 50;
#endif

// ========== ОБРАБОТЧИКИ ПРЕРЫВАНИЙ ЭНКОДЕРОВ ==========
void IRAM_ATTR leftEncISR() {
  static uint8_t last = 0;
  uint8_t cur = (digitalRead(LEFT_ENC_A) << 1) | digitalRead(LEFT_ENC_B);
  if (cur != last) {
    if ((last == 0 && cur == 1) || (last == 1 && cur == 3) ||
        (last == 3 && cur == 2) || (last == 2 && cur == 0))
      left_ticks++;
    else
      left_ticks--;
    last = cur;
  }
}

void IRAM_ATTR rightEncISR() {
  static uint8_t last = 0;
  uint8_t cur = (digitalRead(RIGHT_ENC_A) << 1) | digitalRead(RIGHT_ENC_B);
  if (cur != last) {
    if ((last == 0 && cur == 1) || (last == 1 && cur == 3) ||
        (last == 3 && cur == 2) || (last == 2 && cur == 0))
      right_ticks++;
    else
      right_ticks--;
    last = cur;
  }
}

// ========== ФУНКЦИИ ЭНКОДЕРОВ ==========
long readEncoder(int side) {
  return (side == LEFT) ? left_ticks : right_ticks;
}

void resetEncoder(int side) {
  if (side == LEFT) left_ticks = 0;
  else right_ticks = 0;
}

void resetEncoders() {
  left_ticks = right_ticks = 0;
}

// ========== ФУНКЦИИ МОТОРОВ ==========
void setMotorSpeed(int side, int speed) {
  bool reverse = false;
  if (speed < 0) {
    speed = -speed;
    reverse = true;
  }
  speed = constrain(speed, 0, 255);

  if (side == LEFT) {
    if (!reverse) {
      analogWrite(LEFT_MOTOR_FORWARD, speed);
      analogWrite(LEFT_MOTOR_BACKWARD, 0);
    } else {
      analogWrite(LEFT_MOTOR_BACKWARD, speed);
      analogWrite(LEFT_MOTOR_FORWARD, 0);
    }
  } else {
    if (!reverse) {
      analogWrite(RIGHT_MOTOR_FORWARD, speed);
      analogWrite(RIGHT_MOTOR_BACKWARD, 0);
    } else {
      analogWrite(RIGHT_MOTOR_BACKWARD, speed);
      analogWrite(RIGHT_MOTOR_FORWARD, 0);
    }
  }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}

// ========== PID (опционально) ==========
#ifdef USE_PID
void resetPID() {
  leftPID.target = 0;
  leftPID.prev_enc = readEncoder(LEFT);
  leftPID.output = 0;
  leftPID.prev_input = 0;
  leftPID.iterm = 0;

  rightPID.target = 0;
  rightPID.prev_enc = readEncoder(RIGHT);
  rightPID.output = 0;
  rightPID.prev_input = 0;
  rightPID.iterm = 0;
}

void doPID(SetPointInfo *p, int side) {
  long enc = readEncoder(side);
  int input = enc - p->prev_enc;
  long error = p->target - input;

  long output = (Kp * error - Kd * (input - p->prev_input) + p->iterm) / Ko;
  output += p->output;

  if (output >= MAX_PWM) output = MAX_PWM;
  else if (output <= -MAX_PWM) output = -MAX_PWM;
  else p->iterm += Ki * error;

  p->output = output;
  p->prev_enc = enc;
  p->prev_input = input;
}

void updatePID() {
  if (!moving) {
    if (leftPID.prev_input != 0 || rightPID.prev_input != 0) resetPID();
    return;
  }
  doPID(&leftPID, LEFT);
  doPID(&rightPID, RIGHT);
  setMotorSpeeds(leftPID.output, rightPID.output);
}
#endif

// ========== ПАРСИНГ КОМАНД ==========
void resetCommand() {
  cmd = '\0';
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = arg2 = 0;
  arg = 0;
  idx = 0;
}

void runCommand() {
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch (cmd) {
    case READ_ENCODERS:
      Serial.print(readEncoder(LEFT));
      Serial.print(' ');
      Serial.println(readEncoder(RIGHT));
      break;

    case RESET_ENCODERS:
      resetEncoders();
      Serial.println("OK");
      break;

    case MOTOR_SPEEDS:
      lastMotorCommand = millis();
      if (arg1 == 0 && arg2 == 0) {
        setMotorSpeeds(0, 0);
        moving = false;
      } else {
        moving = true;
      }
      #ifdef USE_PID
        leftPID.target = arg1;
        rightPID.target = arg2;
      #endif
      Serial.println("OK");
      break;

    case MOTOR_RAW_PWM:
      lastMotorCommand = millis();
      moving = false;  // PID отключён
      setMotorSpeeds(arg1, arg2);
      Serial.println("OK");
      break;

    #ifdef USE_PID
    case UPDATE_PID: {
      // Формат: "u Kp:Kd:Ki:Ko"
      char *p = argv1;
      char *tok;
      int i = 0;
      int vals[4];
      while ((tok = strtok_r(p, ":", &p)) != NULL) {
        vals[i++] = atoi(tok);
        if (i >= 4) break;
      }
      if (i == 4) {
        Kp = vals[0]; Kd = vals[1]; Ki = vals[2]; Ko = vals[3];
        Serial.println("OK");
      } else {
        Serial.println("ERROR");
      }
      break;
    }
    #endif

    default:
      Serial.println("Invalid Command");
      break;
  }
}

// ========== SETUP ==========
void setup() {
  Serial.begin(BAUDRATE);

  // Настройка пинов энкодеров
  pinMode(LEFT_ENC_A, INPUT);
  pinMode(LEFT_ENC_B, INPUT);
  pinMode(RIGHT_ENC_A, INPUT);
  pinMode(RIGHT_ENC_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B), leftEncISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), rightEncISR, CHANGE);

  // Настройка пинов моторов
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

  #ifdef USE_PID
    resetPID();
    nextPID = millis() + PID_INTERVAL;
  #endif

  lastMotorCommand = millis();
}

// ========== MAIN LOOP ==========
void loop() {
  // Обработка входящих команд
  while (Serial.available()) {
    chr = Serial.read();

    if (chr == '\r' || chr == '\n') {
      if (arg == 1) argv1[idx] = '\0';
      else if (arg == 2) argv2[idx] = '\0';

      if (cmd != '\0') {
        runCommand();
        resetCommand();
      }

      // Пропускаем все последующие \r и \n
      while (Serial.available()) {
        char c = Serial.peek();
        if (c == '\r' || c == '\n') Serial.read();
        else break;
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

  // PID (если включен)
  #ifdef USE_PID
    if (millis() >= nextPID) {
      updatePID();
      nextPID = millis() + PID_INTERVAL;
    }
  #endif

  // Автоматическая остановка при отсутствии команд (Если используется)
  #ifdef USE_WATCH_DOG
    if (millis() - lastMotorCommand > AUTO_STOP_INTERVAL) {
      setMotorSpeeds(0, 0);
      moving = false;
    }
  #endif
}