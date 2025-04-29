#include <Arduino.h>

// --- Global Pin Definitions ---
// Adjust these pin numbers to match your single motor's wiring
const int MOTOR_PIN1 = 33;     // Motor control pin 1
const int MOTOR_PIN2 = 25;     // Motor control pin 2
const int MOTOR_ENABLE_PIN = 32; // Motor PWM/Enable pin
const int PWM_CHANNEL = 0;     // ESP32 PWM channel (0-15)
const int PWM_FREQ = 30000;    // PWM frequency (Hz)
const int PWM_RESOLUTION = 8;  // PWM resolution (bits, 8 = 0-255)

const uint8_t ENCODER_PINA = 39; // Encoder Channel A pin (usually attach interrupt to this)
const uint8_t ENCODER_PINB = 36; // Encoder Channel B pin

// --- Global State Variables ---
volatile long g_encoder_count = 0; // Stores the encoder ticks
volatile int g_a_last_state = 0;   // Stores the last state of Encoder Pin A

int g_current_speed = 0;       // Current motor speed (0-255)
int g_current_direction = 0;   // Current motor direction (1=forward, -1=backward, 0=stop)

// --- ISR for Encoder (must be fast and simple) ---
// Marked with IRAM_ATTR for ESP32 to run from RAM, improving reliability
void IRAM_ATTR handleEncoder() {
  // Read current state of Pin A
  int a_state = digitalRead(ENCODER_PINA);

  // Check if Pin A state has changed
  if (a_state != g_a_last_state) {
    // If A and B are different, it's one direction; if same, it's the other.
    // This logic depends on how your encoder is wired and its direction.
    // You might need to swap +1 and -1 depending on your physical setup.
    if (digitalRead(ENCODER_PINB) != a_state) {
      g_encoder_count++; // Assume this is forward
    } else {
      g_encoder_count--; // Assume this is backward
    }
  }
  // Update last state of Pin A for the next comparison
  g_a_last_state = a_state;
}

// --- Motor Control Function ---
// direction: 1 for forward, -1 for backward, 0 for stop
// speed: 0-255 for PWM
void controlMotor(int direction, int speed) {
  // Ensure speed is within the valid range
  speed = constrain(speed, 0, 255);

  // Set direction pins based on requested direction
  if (direction == 1) { // Forward
    digitalWrite(MOTOR_PIN1, LOW);
    digitalWrite(MOTOR_PIN2, HIGH);
  } else if (direction == -1) { // Backward
    digitalWrite(MOTOR_PIN1, HIGH);
    digitalWrite(MOTOR_PIN2, LOW);
  } else { // Stop
    digitalWrite(MOTOR_PIN1, LOW);
    digitalWrite(MOTOR_PIN2, LOW);
    speed = 0; // Speed should be 0 when stopped
  }

  // Set the PWM speed
  ledcWrite(MOTOR_ENABLE_PIN, speed);

  // Update global state tracking (optional for this simple demo)
  g_current_speed = speed;
  g_current_direction = direction;
}

// --- Function to Get Encoder Count Safely ---
long getEncoderCount() {
  noInterrupts(); // Disable interrupts temporarily
  long count = g_encoder_count; // Read the volatile global variable
  interrupts();   // Re-enable interrupts
  return count;
}

// --- Function to Reset Encoder Count Safely ---
void resetEncoder() {
  noInterrupts();
  g_encoder_count = 0;
  interrupts();
}

void setup() {
  Serial.begin(115200);
  Serial.println("Simple Single Motor with Encoder Test");

  // --- Setup Motor Pins ---
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);

  // --- Setup PWM Channel for Enable Pin (ESP32 specific) ---
  ledcAttachChannel(MOTOR_ENABLE_PIN,PWM_FREQ,PWM_RESOLUTION, PWM_CHANNEL); // Attach pin to channel
  

  // Ensure motor is stopped initially
  controlMotor(0, 0);

  // --- Setup Encoder Pins ---
  pinMode(ENCODER_PINA, INPUT_PULLUP); // Use internal pull-up resistors
  pinMode(ENCODER_PINB, INPUT_PULLUP);

  // --- Initialize Encoder State ---
  g_a_last_state = digitalRead(ENCODER_PINA); // Read initial state of Pin A
  g_encoder_count = 0;                       // Start count from 0

  // --- Attach Interrupt ---
  // Attach the ISR function 'handleEncoder' to changes on ENCODER_PINA
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA), handleEncoder, CHANGE);
}

void loop() {
  // --- Read and Print Encoder Count ---
  static long last_encoder_count = 0;
  long current_encoder_count = getEncoderCount(); // Use the safe getter function

  // Only print if the count has changed
  if (current_encoder_count != last_encoder_count) {
    Serial.printf("Encoder Count: %ld\n", current_encoder_count);
    last_encoder_count = current_encoder_count;
  }

  // --- Simple Motor Control Sequence ---
  // Cycles through Forward, Stop, Backward, Stop states every 3 seconds
  static unsigned long last_change_time = 0;
  static int state = 0; // 0: Forward, 1: Stop (after FWD), 2: Backward, 3: Stop (after BWD)

  if (millis() - last_change_time > 3000) { // Check if 3 seconds have passed
    last_change_time = millis(); // Update last change time
    state = (state + 1) % 4;     // Move to the next state (0 -> 1 -> 2 -> 3 -> 0 ...)

    switch(state) {
      case 0: // State 0: Move Forward
        Serial.println("\n>> MOTOR FORWARD <<");
        controlMotor(1, 255); // Direction 1 (forward), Speed 150
        break;

      case 1: // State 1: Stop
        Serial.println("\n>> MOTOR STOPPED <<");
        controlMotor(0, 0); // Direction 0 (stop), Speed 0
        break;

      case 2: // State 2: Move Backward
        Serial.println("\n>> MOTOR REVERSE <<");
        controlMotor(-1, 255); // Direction -1 (backward), Speed 150
        break;

      case 3: // State 3: Stop
        Serial.println("\n>> MOTOR STOPPED <<");
        controlMotor(0, 0); // Direction 0 (stop), Speed 0
        break;
    }
  }

  delay(10); // Small delay to prevent loop from running too fast
}