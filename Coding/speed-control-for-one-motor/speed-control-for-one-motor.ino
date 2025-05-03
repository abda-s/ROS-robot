#include <Arduino.h>

// --- Motor Pins ---
const int MOTOR_PIN1 = 33;
const int MOTOR_PIN2 = 25;
const int MOTOR_ENABLE_PIN = 32;
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 30000;
const int PWM_RESOLUTION = 8;

// --- Encoder Pins ---
const uint8_t ENCODER_PINA = 39;
const uint8_t ENCODER_PINB = 36;

// --- Encoder State (4× decoding) ---
volatile long g_encoder_count = 0;

// --- PID Variables ---
float KP = 0.5, KI = 0.58, KD = 0.0;
float g_target_speed_rpm = 30.0;
volatile float g_current_speed_rpm = 0.0;
float e = 0.0, e_prev = 0.0;
float inte = 0.0, inte_prev = 0.0;
float V = 0.0;
const float Vmax = 6.0, Vmin = -6.0;

unsigned long g_pid_last_time_ms = 0;
long g_pid_last_encoder_count = 0;

const int ENCODER_COUNTS_PER_REVOLUTION = 3960;

// --- Timer Variables ---
hw_timer_t *g_pid_timer = nullptr;
volatile unsigned long g_pid_timer_count = 0;
unsigned long g_pid_count_prev = 0;
const unsigned long CONTROL_LOOP_PERIOD_MS = 20;
const uint64_t TIMER_ALARM_VALUE = CONTROL_LOOP_PERIOD_MS * 1000;


// --- ISR for Encoder A ---
void IRAM_ATTR handleEncoderA() {
  bool a = digitalRead(ENCODER_PINA);
  bool b = digitalRead(ENCODER_PINB);
  g_encoder_count += (a == b) ? +1 : -1;
}

// --- ISR for Encoder B ---
void IRAM_ATTR handleEncoderB() {
  bool a = digitalRead(ENCODER_PINA);
  bool b = digitalRead(ENCODER_PINB);
  g_encoder_count += (a != b) ? +1 : -1;
}

// --- PID Timer ISR ---
void IRAM_ATTR onPidTimer() {
  g_pid_timer_count++;
}

// --- Motor Control ---
void controlMotorPWM(int direction, int speed_pwm) {
  speed_pwm = constrain(speed_pwm, 0, 255);
  if (direction > 0) {
    digitalWrite(MOTOR_PIN1, LOW);
    digitalWrite(MOTOR_PIN2, HIGH);
  } else if (direction < 0) {
    digitalWrite(MOTOR_PIN1, HIGH);
    digitalWrite(MOTOR_PIN2, LOW);
  } else {
    digitalWrite(MOTOR_PIN1, LOW);
    digitalWrite(MOTOR_PIN2, LOW);
    speed_pwm = 0;
  }
  ledcWrite(MOTOR_ENABLE_PIN, speed_pwm);
}

// --- Safe Encoder Count Access ---
long getEncoderCount() {
  noInterrupts();
  long c = g_encoder_count;
  interrupts();
  return c;
}

void setup() {
  Serial.begin(115200);

  // Motor PWM
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
  ledcAttachChannel(MOTOR_ENABLE_PIN, PWM_FREQ, PWM_RESOLUTION, PWM_CHANNEL);
  controlMotorPWM(0, 0);

  // Encoder Inputs + ISRs
  pinMode(ENCODER_PINA, INPUT_PULLUP);
  pinMode(ENCODER_PINB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINB), handleEncoderB, CHANGE);

  // --- Setup PID Timer ---

  // timerBegin(frequency): Set timer tick frequency. 1MHz means 1 tick = 1 microsecond.
  g_pid_timer = timerBegin(1000000);

  // timerAttachInterrupt(timer, handler): Attach the ISR function to the timer.
  timerAttachInterrupt(g_pid_timer, &onPidTimer);  // Attaching the counter increment ISR

  // timerAlarm(timer, alarm_value, repeat, count): Set alarm value, repeat mode, and count.
  timerAlarm(g_pid_timer, TIMER_ALARM_VALUE, true, 0);  // Set alarm for periodic trigger



  // Init state
  g_pid_last_time_ms = millis();
  g_pid_last_encoder_count = getEncoderCount();

  Serial.println("Setup complete. Textbook dt (seconds) style.");
}

String g_serial_buffer;
const unsigned long SERIAL_TIMEOUT_MS = 50;

void processSerial() {
  while (Serial.available()) {
    char c = Serial.read();  // Non-blocking read
    Serial.println(g_serial_buffer);
    if (c == 'E') {
      if (g_serial_buffer.length() > 1) {
        char t = g_serial_buffer.charAt(0);
        float v = g_serial_buffer.substring(1).toFloat();
        switch (t) {
          case 'T':
            g_target_speed_rpm = v;
            e_prev = inte_prev = 0.0;
            break;
          case 'P': KP = v; break;
          case 'I': KI = v; break;
          case 'D': KD = v; break;
        }
      }
      g_serial_buffer = "";
    } else if (isPrintable(c)) {
      g_serial_buffer += c;
    }
  }
  // Optional: Timeout for partial commands
  // if (g_serial_buffer.length() > 0 &&
  //     millis() - g_serial_last_received > SERIAL_TIMEOUT_MS) {
  //   g_serial_buffer = "";
  // }
}

void loop() {
  // — Serial command parsing (T=target, P/I/D=gains)
  processSerial();  // Non-blocking

  // — PID every CONTROL_LOOP_PERIOD_MS via hardware timer
  if (g_pid_timer_count > g_pid_count_prev) {
    g_pid_count_prev = g_pid_timer_count;

    // Time delta in seconds
    unsigned long now = millis();
    float dt_sec = (now - g_pid_last_time_ms) / 1000.0f;
    if (dt_sec <= 0) dt_sec = 1e-3;  // guard

    // Encoder delta → revolutions
    long enc_now = getEncoderCount();
    long delta = enc_now - g_pid_last_encoder_count;
    float rev = (float)delta / ENCODER_COUNTS_PER_REVOLUTION;

    // RPM: rev/sec × 60
    g_current_speed_rpm = (rev / dt_sec) * 60.0f;

    // PID terms
    e = g_target_speed_rpm - g_current_speed_rpm;
    inte = inte_prev + e * dt_sec;
    float dedt = (e - e_prev) / dt_sec;
    V = KP * e + KI * inte + KD * dedt;

    // Anti-windup
    if (V > Vmax) V = Vmax;
    else if (V < Vmin) V = Vmin;

    // Apply
    int dir = (V > 0) ? 1 : (V < 0) ? -1
                                    : 0;
    int pwm = constrain(int(fabs(V) / Vmax * 255.0f), 0, 255);
    controlMotorPWM(dir, pwm);

    // Debug
    Serial.printf("P:%.2f, I:%.2f, D:%.4f, RPM:%.2f,target:%.2f,PWM:%d, inte:%.3f\n",
                  KP, KI, KD, g_current_speed_rpm, g_target_speed_rpm, pwm, inte);

    // Save state
    g_pid_last_time_ms = now;
    g_pid_last_encoder_count = enc_now;
    e_prev = e;
    inte_prev = inte;
  }
}
