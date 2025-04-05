#include <Arduino.h>

class MotorWithEncoder {
private:
  // Motor control pins
  int pin1_;
  int pin2_;
  int enablePin_;
  int channel_;
  
  // Encoder pins and state
  uint8_t encoderPinA_;
  uint8_t encoderPinB_;
  volatile long encoderCounter_;  // Changed from int to long
  volatile int aLastState_;
  
  // Motor state
  int currentSpeed_;
  int currentDirection_; // 1=forward, -1=backward, 0=stop

public:
  // Constructor
  MotorWithEncoder(int motorPin1, int motorPin2, int enablePin, int channel,
                 uint8_t encoderPinA, uint8_t encoderPinB)
    : pin1_(motorPin1), pin2_(motorPin2), enablePin_(enablePin), channel_(channel),
      encoderPinA_(encoderPinA), encoderPinB_(encoderPinB), 
      encoderCounter_(0), aLastState_(0),
      currentSpeed_(0), currentDirection_(0) {
    
    // Setup motor pins
    pinMode(pin1_, OUTPUT);
    pinMode(pin2_, OUTPUT);
    ledcAttachChannel(enablePin_, 30000, 8, channel_);
    
    // Setup encoder pins
    pinMode(encoderPinA_, INPUT_PULLUP);
    pinMode(encoderPinB_, INPUT_PULLUP);
    aLastState_ = digitalRead(encoderPinA_);
  }

  // Encoder update function to be called from ISR
  void updateEncoder() {
    int aState = digitalRead(encoderPinA_);
    if (aState != aLastState_) {
      if (digitalRead(encoderPinB_) != aState) {
        encoderCounter_++;
      } else {
        encoderCounter_--;
      }
    }
    aLastState_ = aState;
  }

  // Move the motor with specified direction and speed
  void move(int direction, int speed) {
    speed = constrain(speed, 0, 255); // Ensure speed is within PWM range
    
    if (direction == 1) { // Forward
      digitalWrite(pin1_, LOW);
      digitalWrite(pin2_, HIGH);
    } else if (direction == -1) { // Backward
      digitalWrite(pin1_, HIGH);
      digitalWrite(pin2_, LOW);
    } else { // Stop
      digitalWrite(pin1_, LOW);
      digitalWrite(pin2_, LOW);
      speed = 0;
    }
    
    ledcWrite(enablePin_, speed);
    currentSpeed_ = speed;
    currentDirection_ = direction;
  }

  // Stop the motor
  void stop() {
    move(0, 0);
  }

  // Get current speed
  int getSpeed() const {
    return currentSpeed_;
  }

  // Get current direction
  int getDirection() const {
    return currentDirection_;
  }

  // Get encoder count (atomically)
  long getEncoderCount() const {  // Changed return type to long
    noInterrupts();
    long count = encoderCounter_;
    interrupts();
    return count;
  }

  // Reset encoder count
  void resetEncoder() {
    noInterrupts();
    encoderCounter_ = 0;
    interrupts();
  }

  // Get encoder pin A for interrupt attachment
  uint8_t getEncoderPinA() const {
    return encoderPinA_;
  }
};

class Side {
private:
  MotorWithEncoder& motor1_;
  MotorWithEncoder& motor2_;
  
public:
  Side(MotorWithEncoder& m1, MotorWithEncoder& m2) 
    : motor1_(m1), motor2_(m2) {}

  void setSpeed(int speed) {
    // Convert signed speed to direction + absolute speed
    int direction = (speed > 0) ? 1 : (speed < 0) ? -1 : 0;
    int absSpeed = constrain(abs(speed), 0, 255);
    
    motor1_.move(direction, absSpeed);
    motor2_.move(direction, absSpeed);
  }

  void stop() {
    motor1_.stop();
    motor2_.stop();
  }

  long getAverageEncoder() {
    return (motor1_.getEncoderCount() + motor2_.getEncoderCount()) / 2;
  }
};

class DifferentialDrive {
private:
  Side& left_;
  Side& right_;
  
public:
  DifferentialDrive(Side& left, Side& right)
    : left_(left), right_(right) {}

  void tankDrive(int leftSpeed, int rightSpeed) {
    left_.setSpeed(leftSpeed);
    right_.setSpeed(rightSpeed);
  }

  void arcadeDrive(int throttle, int steering) {
    int left = constrain(throttle + steering, -255, 255);
    int right = constrain(throttle - steering, -255, 255);
    tankDrive(left, right);
  }
};



// Create motor+encoder objects
MotorWithEncoder motor1(13, 12, 14, 0, 17, 16);  // Motor 1 with encoder
MotorWithEncoder motor2(33, 25, 32, 1, 39, 36);  // Motor 2 with encoder
MotorWithEncoder motor3(18, 19, 5, 2, 26, 27);   // Motor 3 with encoder
MotorWithEncoder motor4(2, 15, 4, 3, 35, 34);    // Motor 4 with encoder

// Group motors into sides (adjust pairs according to your robot's wiring)
Side leftSide(motor2, motor4);  // Front-left + Rear-left
Side rightSide(motor1, motor3); // Front-right + Rear-right
DifferentialDrive robot(leftSide, rightSide);



// Individual ISR handlers for each encoder
void IRAM_ATTR handleEncoder0() {
  motor1.updateEncoder();
}
void IRAM_ATTR handleEncoder1() {
  motor2.updateEncoder();
}
void IRAM_ATTR handleEncoder2() {
  motor3.updateEncoder();
}
void IRAM_ATTR handleEncoder3() {
  motor4.updateEncoder();
}

void setup() {
  Serial.begin(115200);
  Serial.println("Testing Motors with Encoders...");

  // Attach interrupts for each encoder
  attachInterrupt(digitalPinToInterrupt(motor1.getEncoderPinA()), handleEncoder0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor2.getEncoderPinA()), handleEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor3.getEncoderPinA()), handleEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor4.getEncoderPinA()), handleEncoder3, CHANGE);
}

void printEncoderPositions(long positions[4]) {  // Changed parameter type to long

  // Print all motor positions in one line with proper formatting
  Serial.printf("%9ld | %9ld | %9ld | %9ld\n",
               positions[0], positions[1], 
               positions[2], positions[3]);
}

void loop() {
  static long lastCounters[4] = {0};  // Changed to long
  long currentCounters[4];           // Changed to long
  
  // Read all counters atomically
  currentCounters[0] = motor1.getEncoderCount();
  currentCounters[1] = motor2.getEncoderCount();
  currentCounters[2] = motor3.getEncoderCount();
  currentCounters[3] = motor4.getEncoderCount();

  // Print encoder changes in formatted way
  bool anyChange = false;
  for (int i = 0; i < 4; i++) {
    if (currentCounters[i] != lastCounters[i]) {
      anyChange = true;
      lastCounters[i] = currentCounters[i];
    }
  }
  
  if (anyChange) {
    printEncoderPositions(currentCounters);
  }

   // Modified motor control sequence
  static unsigned long lastChange = 0;
  static int state = 0;
  
  if (millis() - lastChange > 2000) {
    lastChange = millis();
    state = (state + 1) % 3;
    
    switch(state) {
      case 0: // Forward
        Serial.println("\n>> ALL MOTORS FORWARD <<");
        robot.tankDrive(255, 255);
        break;
        
      case 1: // Stop
        Serial.println("\n>> MOTORS STOPPED <<");
        robot.tankDrive(0, 0);
        break;
        
      case 2: // Backward
        Serial.println("\n>> ALL MOTORS REVERSE <<");
        robot.tankDrive(-255, -255);
        break;
    }
  }

  delay(10);
}