#include <Arduino.h>
#include <string.h>
#include <Wire.h> // Required for I2C communication
#include <LiquidCrystal_I2C.h> // Required for I2C LCD
#include <cmath> // Required for fabs

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
const unsigned long CONTROL_LOOP_PERIOD_MS = 10; // PID loop period
const uint64_t TIMER_ALARM_VALUE = CONTROL_LOOP_PERIOD_MS * 1000; // in microseconds

// --- Serial Communication Protocol ---
#define TERMINATOR_1 0xAA // 170 in dec
#define TERMINATOR_2 0x55 // 85 in dec
const size_t DOUBLE_SIZE = sizeof(double);
const size_t PACKET_SIZE = DOUBLE_SIZE + 2; // Size of double + 2 terminators
const size_t READ_BUFFER_SIZE = PACKET_SIZE * 4;
byte readBuffer[READ_BUFFER_SIZE];
size_t readBufferIndex = 0;

// --- LCD Configuration ---
// Adjust these based on your specific LCD
const int LCD_COLS = 16; // Usually 16 or 20
const int LCD_ROWS = 2;  // Usually 2 or 4
// Find your LCD's I2C address using an I2C scanner sketch if you don't know it.
// Common addresses are 0x27 or 0x3F.
const int LCD_I2C_ADDRESS = 0x27;

// Initialize the LCD object
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLS, LCD_ROWS);


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
  ledcWrite(MOTOR_ENABLE_PIN, speed_pwm); // Use the PWM_CHANNEL define
}

// --- Safe Encoder Count Access ---
long getEncoderCount() {
  noInterrupts();
  long c = g_encoder_count;
  interrupts();
  return c;
}

// --- Double Precision Serial Communication Functions ---
double readDoubleFromSerial() {
  // Read available bytes into the buffer
  while (Serial.available() && readBufferIndex < READ_BUFFER_SIZE) {
    byte incomingByte = Serial.read();
    readBuffer[readBufferIndex++] = incomingByte;
  }

  // Search for a complete packet in the buffer
  for (size_t i = 0; i + PACKET_SIZE <= readBufferIndex; ++i) {
    // Check if we have a valid packet with terminators in the right position
    if (readBuffer[i + DOUBLE_SIZE] == TERMINATOR_1 &&
        readBuffer[i + DOUBLE_SIZE + 1] == TERMINATOR_2) {

      // Found a valid packet, extract the double value
      double result;
      // Endianness Consideration: Same as before, verify if needed for your systems.
      memcpy(&result, &readBuffer[i], DOUBLE_SIZE);

      // Move any remaining bytes to the beginning of the buffer
      size_t remainingBytes = readBufferIndex - (i + PACKET_SIZE);
      if (remainingBytes > 0) {
        memmove(readBuffer, &readBuffer[i + PACKET_SIZE], remainingBytes);
      }
      readBufferIndex = remainingBytes;

      return result;
    }
  }

  // If buffer is full but no valid packet found, discard oldest data
  // Added a check to prevent discarding from an empty or nearly empty buffer
  if (readBufferIndex >= READ_BUFFER_SIZE - PACKET_SIZE) { // Check if buffer is almost full
    const size_t discard_amount = PACKET_SIZE; // Discard one packet's worth
     if (readBufferIndex >= discard_amount) { // Only discard if enough data exists
        memmove(readBuffer, &readBuffer[discard_amount], readBufferIndex - discard_amount);
        readBufferIndex -= discard_amount;
     } else { // Fallback if buffer is unexpectedly small but full
         readBufferIndex = 0;
     }
  }

  return NAN;  // No complete packet found
}

void writeDoubleToSerial(double number) {
  byte doubleBytes[DOUBLE_SIZE];
  memcpy(doubleBytes, &number, DOUBLE_SIZE);
  Serial.write(doubleBytes, DOUBLE_SIZE);
  Serial.write(TERMINATOR_1);
  Serial.write(TERMINATOR_2);
}

void setup() {
  Serial.begin(115200);

  // --- Initialize LCD ---
  Wire.begin(); // Initialize I2C communication (ESP32 default pins for Wire are GPIO 21 (SDA) and GPIO 22 (SCL))
  lcd.init();   // Initialize the lcd
  lcd.backlight(); // Turn on the backlight
  lcd.setCursor(0, 0);
  lcd.print("ESP32 Motor Ctrl");
  lcd.setCursor(0, 1);
  lcd.print("Init...");


  // Motor PWM
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT); // Keep as OUTPUT, but PWM uses the channel
  ledcAttachChannel(MOTOR_ENABLE_PIN, PWM_FREQ, PWM_RESOLUTION, PWM_CHANNEL); // Attach PWM to channel, not the pin directly

  controlMotorPWM(0, 0);

  // Encoder Inputs + ISRs
  pinMode(ENCODER_PINA, INPUT_PULLUP);
  pinMode(ENCODER_PINB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINB), handleEncoderB, CHANGE);

  // --- Setup PID Timer ---

  // timerBegin(frequency): Set timer tick frequency. 1MHz means 1 tick = 1 microsecond
  g_pid_timer = timerBegin(1000000);
  
  // timerAttachInterrupt(timer, handler): Attach the ISR function to the timer
  timerAttachInterrupt(g_pid_timer, &onPidTimer);
  
  // timerAlarm(timer, alarm_value, repeat, count): Set alarm value, repeat mode, and enable
  // TIMER_ALARM_VALUE is in microseconds (since we set frequency to 1MHz)
  // true = autoreload, 0 = unlimited repeats
  timerAlarm(g_pid_timer, TIMER_ALARM_VALUE, true, 0);


  // Init state
  g_pid_last_time_ms = millis();
  g_pid_last_encoder_count = getEncoderCount();

  Serial.println("Motor controller initialized with double-precision serial protocol");
}

void loop() {
  // --- Read motor command using the double protocol ---
  double V_command = readDoubleFromSerial();

  if (!isnan(V_command)) { // If we received a valid value
    // Convert V_command (e.g., -6.0 to 6.0) to direction and PWM (0 to 255)
    int dir = 0;
    int pwm = 0;

    if (V_command > 0) {
        dir = 1;
    } else if (V_command < 0) {
        dir = -1;
    }

    // Map the absolute value of V_command to the PWM range [0, 255]
    pwm = constrain(int(fabs(V_command) / Vmax * 255.0), 0, 255); // Use double for calculation
    controlMotorPWM(dir, pwm);

    // Optional: Display received command on LCD briefly
    // lcd.clear();
    // lcd.setCursor(0, 0);
    // lcd.print("Cmd: ");
    // lcd.print(V_command, 2);
  }

  // --- Speed Calculation and Send Data Back ---
  // This block executes every CONTROL_LOOP_PERIOD_MS based on the timer ISR
  if (g_pid_timer_count > g_pid_count_prev) {
    g_pid_count_prev = g_pid_timer_count;

    unsigned long now = millis();
    float dt_sec = (now - g_pid_last_time_ms) / 1000.0f;
    if (dt_sec <= 0) dt_sec = 1e-3; // guard against division by zero

    long enc_now = getEncoderCount();
    long delta = enc_now - g_pid_last_encoder_count;
    float rev = (float)delta / ENCODER_COUNTS_PER_REVOLUTION;

    g_current_speed_rpm = (rev / dt_sec) * 60.0f;

    // --- Display current RPM on LCD ---
    lcd.clear(); // Clear previous RPM value
    lcd.setCursor(0, 0);
    lcd.print("Current RPM:"); // Label
    lcd.setCursor(0, 1);
    // Print the RPM value on the second line
    lcd.print(g_current_speed_rpm, 2); // Print float with 2 decimal places


    // --- Send measured speed using the double protocol ---
    double speed_to_send = (double)g_current_speed_rpm;
    writeDoubleToSerial(speed_to_send);

    // Save state for next calculation
    g_pid_last_time_ms = now;
    g_pid_last_encoder_count = enc_now;
  }
}