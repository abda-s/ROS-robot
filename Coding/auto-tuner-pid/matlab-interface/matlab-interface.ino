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
volatile float g_current_speed_rpm = 0.0;
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

  // Serial.println("Setup complete. Textbook dt (seconds) style.");
}

void loop() {
  // --- Read raw motor command (float) from Serial ---
  // Check if the expected number of bytes for a float is available
  if (Serial.available() >= sizeof(float)) {
    float V_command;
    // Read the raw bytes directly into the float variable
    Serial.readBytes((byte*)&V_command, sizeof(V_command));

    // Convert V_command (e.g., -6.0 to 6.0) to direction and PWM (0 to 255)
    // Ensure Vmax and Vmin are defined globally and correctly represent
    // the voltage range you want to map to the full PWM range.
    int dir = 0;
    int pwm = 0;
    if (V_command > 0) {
        dir = 1;
    } else if (V_command < 0) {
        dir = -1;
    }

    // Map the absolute value of V_command to the PWM range [0, 255]
    // Using Vmax to scale the command.
    pwm = constrain(int(fabs(V_command) / Vmax * 255.0f), 0, 255);


    controlMotorPWM(dir, pwm);

    // Optional: Print received command for debugging (will be slow, use with caution)
    // Serial.printf("Received raw V_command: %.2f, PWM: %d\n", V_command, pwm);
  }

  // --- Speed Calculation (Keep this) ---
  // PID Timer ISR increments g_pid_timer_count periodically
  if (g_pid_timer_count > g_pid_count_prev) {
    g_pid_count_prev = g_pid_timer_count;

    unsigned long now = millis();
    float dt_sec = (now - g_pid_last_time_ms) / 1000.0f;
    if (dt_sec <= 0) dt_sec = 1e-3; // guard against division by zero

    long enc_now = getEncoderCount();
    long delta = enc_now - g_pid_last_encoder_count;
    float rev = (float)delta / ENCODER_COUNTS_PER_REVOLUTION;

    g_current_speed_rpm = (rev / dt_sec) * 60.0f;

    // --- Send measured speed (raw float) back to Serial ---
    // Send the raw bytes of the g_current_speed_rpm float variable
// Serial.write((uint8_t*)&g_current_speed_rpm, sizeof(g_current_speed_rpm));

    // Optional: Print sent data for debugging (will interfere with raw data!)
    Serial.println(g_current_speed_rpm);



    // Save state for next calculation
    g_pid_last_time_ms = now;
    g_pid_last_encoder_count = enc_now;
  }
}
