#include <Arduino.h>
#include <functional>  // Required for std::function

// Define a class to manage a single motor
class MotorController {
public:
  // Constructor
  MotorController(int motorPin1, int motorPin2, int motorEnablePin,
                  int pwmChannel,
                  uint8_t encoderPinA, uint8_t encoderPinB)
    : _motorPin1(motorPin1), _motorPin2(motorPin2), _motorEnablePin(motorEnablePin),
      _pwmChannel(pwmChannel),
      _encoderPinA(encoderPinA), _encoderPinB(encoderPinB) {
    // Initialize PID variables
    _g_encoder_count = 0;
    _g_target_speed_rpm = 0.0;  // Start at 0 target speed
    _g_current_speed_rpm = 0.0;
    _e = 0.0;
    _e_prev = 0.0;
    _inte = 0.0;
    _inte_prev = 0.0;
    _V = 0.0;
    _Vmax = 6.0;  // Assuming a voltage range, adjust if needed
    _Vmin = -6.0;
    _g_pid_last_time_ms = 0;
    _g_pid_last_encoder_count = 0;

    _pwmFreq = 3000;
    _pwmResolution = 8;
    _KP = 0.5;
    _KI = 0.58;
    _KD = 0;
    _encoderCountsPerRevolution = 3960;
  }

  // Call this in your Arduino setup() function
  void begin() {
    // Motor PWM Setup
    pinMode(_motorPin1, OUTPUT);
    pinMode(_motorPin2, OUTPUT);
    ledcAttachChannel(_motorEnablePin, _pwmFreq, _pwmResolution, _pwmChannel);
    controlMotorPWM(0, 0);  // Start motor off

    // Encoder Inputs + ISRs
    pinMode(_encoderPinA, INPUT_PULLUP);
    pinMode(_encoderPinB, INPUT_PULLUP);

    // Store a pointer to this instance so the static ISRs can find it
    // This requires a static mechanism to map pins or channels to instances.
    // A simple approach for a few motors is to pass 'this' pointer if attachInterrupt allows,
    // or use a static map if we have many motors and need dynamic mapping.
    // ESP32's attachInterrupt allows passing a void* argument.
    attachInterruptArg(digitalPinToInterrupt(_encoderPinA), handleEncoderA_static, this, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(_encoderPinB), handleEncoderB_static, this, CHANGE);

    // Initialize PID timing state
    _g_pid_last_time_ms = millis();
    _g_pid_last_encoder_count = getEncoderCount();
  }

  // Call this periodically from your main loop() - triggered by a timer
  void updatePID(unsigned long currentTime_ms) {
    // Time delta in seconds
    float dt_sec = (currentTime_ms - _g_pid_last_time_ms) / 1000.0f;
    if (dt_sec <= 0) dt_sec = 1e-3;  // guard

    // Encoder delta → revolutions
    long enc_now = getEncoderCount();
    long delta = enc_now - _g_pid_last_encoder_count;
    float rev = (float)delta / _encoderCountsPerRevolution;

    // RPM: rev/sec × 60
    _g_current_speed_rpm = (rev / dt_sec) * 60.0f;

    // PID terms
    _e = _g_target_speed_rpm - _g_current_speed_rpm;
    _inte = _inte_prev + _e * dt_sec;
    float dedt = (_e - _e_prev) / dt_sec;
    _V = _KP * _e + _KI * _inte + _KD * dedt;

    // Anti-windup
    if (_V > _Vmax) _V = _Vmax;
    else if (_V < _Vmin) _V = _Vmin;

    // Apply motor command
    int dir = (_V > 0) ? 1 : (_V < 0) ? -1
                                      : 0;
    int pwm = constrain(int(fabs(_V) / _Vmax * 255.0f), 0, 255);
    pwm = (_g_target_speed_rpm == 0) ? 0 : pwm; // TO NOT MAKE THE MOTORS MAKE SOUNDS
    controlMotorPWM(dir, pwm);



    // Save state for next iteration
    _g_pid_last_time_ms = currentTime_ms;
    _g_pid_last_encoder_count = enc_now;
    _e_prev = _e;
    _inte_prev = _inte;
  }

  // --- Getters and Setters ---
  void setTargetSpeedRPM(float target_rpm) {
    _g_target_speed_rpm = target_rpm;
    // Reset integral and error history when target changes significantly
    _e_prev = 0.0;
    _inte_prev = 0.0;
    _inte = 0.0;  // Also reset current integral
  }

  float getTargetSpeedRPM() const {
    return _g_target_speed_rpm;
  }
  float getCurrentSpeedRPM() const {
    return _g_current_speed_rpm;
  }
  long getEncoderCount() {
    noInterrupts();
    long c = _g_encoder_count;
    interrupts();
    return c;
  }

  void setPIDGains(float kp, float ki, float kd) {
    _KP = kp;
    _KI = ki;
    _KD = kd;
  }

  float getKP() const {
    return _KP;
  }
  float getKI() const {
    return _KI;
  }
  float getKD() const {
    return _KD;
  }

private:
  // --- Motor Pins ---
  int _motorPin1;
  int _motorPin2;
  int _motorEnablePin;
  int _pwmChannel;
  int _pwmFreq;
  int _pwmResolution;

  // --- Encoder Pins ---
  uint8_t _encoderPinA;
  uint8_t _encoderPinB;
  int _encoderCountsPerRevolution;

  // --- Encoder State (4× decoding) ---
  volatile long _g_encoder_count;  // Use _ prefix for member variables

  // --- PID Variables ---
  float _KP, _KI, _KD;
  float _g_target_speed_rpm;
  volatile float _g_current_speed_rpm;
  float _e, _e_prev;
  float _inte, _inte_prev;
  float _V;
  float _Vmax, _Vmin;

  unsigned long _g_pid_last_time_ms;
  long _g_pid_last_encoder_count;

  // --- Motor Control ---
  void controlMotorPWM(int direction, int speed_pwm) {
    speed_pwm = constrain(speed_pwm, 0, 255);
    if (direction > 0) {
      digitalWrite(_motorPin1, LOW);
      digitalWrite(_motorPin2, HIGH);
    } else if (direction < 0) {
      digitalWrite(_motorPin1, HIGH);
      digitalWrite(_motorPin2, LOW);
    } else {
      digitalWrite(_motorPin1, LOW);
      digitalWrite(_motorPin2, LOW);
      speed_pwm = 0;  // Ensure PWM is 0 when stopped
    }
    ledcWrite(_motorEnablePin, speed_pwm);  
  }

  // --- ISRs (Static members to be used with attachInterruptArg) ---
  // These static functions receive a pointer to the instance and call the
  // actual non-static member function.
  static void IRAM_ATTR handleEncoderA_static(void* arg) {
    // Cast the void* argument back to a MotorController pointer
    MotorController* instance = static_cast<MotorController*>(arg);
    if (instance) {
      instance->handleEncoderA_instance();
    }
  }

  static void IRAM_ATTR handleEncoderB_static(void* arg) {
    MotorController* instance = static_cast<MotorController*>(arg);
    if (instance) {
      instance->handleEncoderB_instance();
    }
  }

  // Non-static ISR handlers - called by the static handlers
  void IRAM_ATTR handleEncoderA_instance() {
    bool a = digitalRead(_encoderPinA);
    bool b = digitalRead(_encoderPinB);
    _g_encoder_count += (a == b) ? +1 : -1;
  }

  void IRAM_ATTR handleEncoderB_instance() {
    bool a = digitalRead(_encoderPinA);
    bool b = digitalRead(_encoderPinB);
    _g_encoder_count += (a != b) ? +1 : -1;
  }
};

// --- Global timer setup (can trigger updates for multiple motors) ---
hw_timer_t* g_pid_timer = nullptr;
volatile unsigned long g_pid_timer_count = 0;
unsigned long g_pid_count_prev = 0;
const unsigned long CONTROL_LOOP_PERIOD_MS = 20;
const uint64_t TIMER_ALARM_VALUE = CONTROL_LOOP_PERIOD_MS * 1000;  // in microseconds

// --- Timer ISR (increments a global counter) ---
void IRAM_ATTR onPidTimer() {
  g_pid_timer_count++;
}

// --- Motor Instances ---
// Create instances of the MotorController class for each motor you have.
// Example: One motor
MotorController motor2(33, 25, 32,  // Motor Pins (PIN1, PIN2, ENABLE)
                       0,           // PWM Channel
                       39, 36);     // Encoder Pins (PINA, PINB)

MotorController motor4(2, 15, 4,  // Motor Pins (PIN1, PIN2, ENABLE)
                       1,         // PWM Channel
                       35, 34);   // Encoder Pins (PINA, PINB)

MotorController motor3(18, 19, 5,  // Motor Pins (PIN1, PIN2, ENABLE)
                       2,         // PWM Channel
                       27, 26);   // Encoder Pins (PINA, PINB)

// Example: If you had a second motor, you would create another instance:
// MotorController motor2(PIN1_M2, PIN2_M2, ENABLE_M2, 1, 30000, 8, ENCA_M2, ENCB_M2, 3960, 0.5, 0.58, 0.0);

// Array of motor controllers
MotorController* motors[] = { &motor2, &motor4, &motor3 };
const int NUM_MOTORS = sizeof(motors) / sizeof(motors[0]);

// --- Serial command parsing (Global) ---
String g_serial_buffer;
const unsigned long SERIAL_TIMEOUT_MS = 50;  // Not used in this non-blocking example, but good to keep in mind

// Function to process serial commands for all motors in an array
void processSerialCommands(MotorController* motorArray[], int numMotors) {
  while (Serial.available()) {
    char c = Serial.read();  // Non-blocking read

    if (c == 'E') {  // End character for a command
      if (g_serial_buffer.length() > 1) {
        // Expected format: <command_type><value>E
        char commandType = g_serial_buffer.charAt(0);
        String valueString = g_serial_buffer.substring(1);

        if (valueString.length() > 0) {
          float value = valueString.toFloat();

          switch (commandType) {
            case 'T':
              // Apply the target speed to ALL motors
              for (int i = 0; i < numMotors; i++) {
                motorArray[i]->setTargetSpeedRPM(value);
              }
              Serial.printf("Set Target Speed for ALL motors to %.2f RPM\n", value);
              break;
            default:
              Serial.println("Unknown command type");
              break;
          }
        } else {
          Serial.println("Command received but no value provided.");
        }
      } else {
        Serial.println("Incomplete command received.");
      }
      g_serial_buffer = "";  // Clear buffer after processing
    } else if (isPrintable(c)) {
      g_serial_buffer += c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Motor Controller Setup...");

  // Initialize motor instances
  motor2.begin();
  motor4.begin();
  motor3.begin();

  // If you had motor2: motor2.begin();

  // --- Setup PID Timer ---
  // timerBegin(frequency): Set timer tick frequency. 1MHz means 1 tick = 1 microsecond.
  g_pid_timer = timerBegin(1000000);

  // timerAttachInterrupt(timer, handler): Attach the ISR function to the timer.
  // The third parameter (true) sets the interrupt to edge-triggered (not needed here, level is fine)
  // The fourth parameter (true) enables auto-reload.
  timerAttachInterrupt(g_pid_timer, &onPidTimer);

  // timerAlarm(timer, alarm_value, repeat): Set alarm value and repeat mode.
  timerAlarm(g_pid_timer, TIMER_ALARM_VALUE, true, 0);  // Set alarm for periodic trigger


  Serial.println("Setup complete.");
}

void loop() {
  // --- Serial command parsing (T=target, P/I/D=gains) ---
  // Need to modify this to handle commands for multiple motors if applicable.
  // For now, it will control motor1.
  processSerialCommands(motors, NUM_MOTORS);
  // If you had motor2 and a command format for it, you'd add:
  // processSerialCommands(motor2); // Requires modifying processSerialCommands to handle motor ID

  // --- PID Update every CONTROL_LOOP_PERIOD_MS via hardware timer ---
  if (g_pid_timer_count > g_pid_count_prev) {
    // A PID interval has passed. Update all motor controllers.
    unsigned long now = millis();          // Get time once for this interval
    g_pid_count_prev = g_pid_timer_count;  // Acknowledge timer tick

    // Update motor instances
    motor2.updatePID(now);
    motor3.updatePID(now);
    motor4.updatePID(now);
    // If you had motor2: motor2.updatePID(now);
    Serial.printf("ZERO:0.0, target:%.2f, motor2.RPM:%.2f, motor3.RPM:%.2f, motor4.RPM:%.2f, millis():%d \n",
                  motor2.getTargetSpeedRPM(), motor2.getCurrentSpeedRPM(), motor3.getCurrentSpeedRPM(), motor4.getCurrentSpeedRPM(),millis() );
    // Debug print for overall loop timing (optional)
    // static unsigned long last_debug_time = 0;
    // if (now - last_debug_time >= 1000) { // Print every second
    //     Serial.printf("Loop running. Timer count: %lu\n", g_pid_timer_count);
    //     last_debug_time = now;
    // }
  }

  // Other non-blocking tasks can go here
}