/*********
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Modified for four motors with OOP approach
*********/

class DCMotor {
private:
  int pin1;
  int pin2;
  int enablePin;
  int channel;
  int currentSpeed;
  int currentDirection; // 1=forward, -1=backward, 0=stop

public:
  // Constructor
  DCMotor(int p1, int p2, int en, int ch) 
    : pin1(p1), pin2(p2), enablePin(en), channel(ch), 
      currentSpeed(0), currentDirection(0) {
    // Set pins as outputs
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    
    // Configure LEDC PWM for this motor
    ledcAttachChannel(enablePin, 30000, 8, channel);
  }

  // Move the motor with specified direction and speed
  void move(int direction, int speed) {
    speed = constrain(speed, 0, 255); // Ensure speed is within PWM range
    
    if (direction == 1) { // Forward
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, HIGH);
    } else if (direction == -1) { // Backward
      digitalWrite(pin1, HIGH);
      digitalWrite(pin2, LOW);
    } else { // Stop
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, LOW);
      speed = 0;
    }
    
    ledcWrite(enablePin, speed);
    currentSpeed = speed;
    currentDirection = direction;
  }

  // Stop the motor
  void stop() {
    move(0, 0);
  }

  // Get current speed
  int getSpeed() const {
    return currentSpeed;
  }

  // Get current direction
  int getDirection() const {
    return currentDirection;
  }
};

// Create motor objects with their respective pins and channels
DCMotor motor1(13, 12, 14, 0);  // Motor A (Motor 1)
DCMotor motor2(33, 25, 32, 1);  // Motor B (Motor 2)
DCMotor motor3(18, 19, 5, 2);   // Motor C (Motor 3)
DCMotor motor4(2, 15, 4, 3);    // Motor D (Motor 4)

// PWM properties
const int dutyCycle = 255;

void setup() {
  Serial.begin(115200);
  Serial.println("Testing DC Motors with OOP...");
  
  // Motor initialization is handled in their constructors
}

void loop() {
  // Move all motors forward
  Serial.println("Moving Forward");
  motor1.move(1, dutyCycle);
  motor2.move(1, dutyCycle);
  motor3.move(1, dutyCycle);
  motor4.move(1, dutyCycle);
  delay(2000);

  // Stop all motors
  Serial.println("Motors Stopped");
  motor1.stop();
  motor2.stop();
  motor3.stop();
  motor4.stop();
  delay(1000);

  // Move all motors backward
  Serial.println("Moving Backward");
  motor1.move(-1, dutyCycle);
  motor2.move(-1, dutyCycle);
  motor3.move(-1, dutyCycle);
  motor4.move(-1, dutyCycle);
  delay(2000);

  // Stop all motors
  Serial.println("Motors Stopped");
  motor1.stop();
  motor2.stop();
  motor3.stop();
  motor4.stop();
  delay(1000);
}