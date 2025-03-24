/*********
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Modified for four motors
*********/

// Motor A (Motor 1)
int motor1Pin1 = 13;
int motor1Pin2 = 12;
int enable1Pin = 14;

// Motor B (Motor 2)
int motor2Pin1 = 33;
int motor2Pin2 = 25;
int enable2Pin = 32;

// Motor C (Motor 3)
int motor3Pin1 = 18;
int motor3Pin2 = 19;
int enable3Pin = 5;

// Motor D (Motor 4)
int motor4Pin1 = 2;
int motor4Pin2 = 15;
int enable4Pin = 4;

// PWM properties
const int freq = 30000;
const int resolution = 8;
int dutyCycle = 255;

void setup() {
  Serial.begin(115200);
  Serial.println("Testing DC Motors...");

  // Set motor pins as outputs
  int motorPins[] = {motor1Pin1, motor1Pin2, enable1Pin, 
                     motor2Pin1, motor2Pin2, enable2Pin, 
                     motor3Pin1, motor3Pin2, enable3Pin, 
                     motor4Pin1, motor4Pin2, enable4Pin};
  
  for (int i = 0; i < 12; i++) {
    pinMode(motorPins[i], OUTPUT);
  }
  
  // Configure LEDC PWM for each motor
  ledcAttachChannel(enable1Pin, freq, resolution, 0);
  ledcAttachChannel(enable2Pin, freq, resolution, 1);
  ledcAttachChannel(enable3Pin, freq, resolution, 2);
  ledcAttachChannel(enable4Pin, freq, resolution, 3);
}

void moveMotor(int motorPin1, int motorPin2, int enablePin, int direction, int speed) {
  if (direction == 1) { // Forward
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  } else if (direction == -1) { // Backward
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  } else { // Stop
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
  }
  ledcWrite(enablePin, speed);
}

void loop() {
  // Move all motors forward
  Serial.println("Moving Forward");
  moveMotor(motor1Pin1, motor1Pin2, enable1Pin, 1, dutyCycle);
  moveMotor(motor2Pin1, motor2Pin2, enable2Pin, 1, dutyCycle);
  moveMotor(motor3Pin1, motor3Pin2, enable3Pin, 1, dutyCycle);
  moveMotor(motor4Pin1, motor4Pin2, enable4Pin, 1, dutyCycle);
  delay(2000);

  // Stop all motors
  Serial.println("Motors Stopped");
  moveMotor(motor1Pin1, motor1Pin2, enable1Pin, 0, 0);
  moveMotor(motor2Pin1, motor2Pin2, enable2Pin, 0, 0);
  moveMotor(motor3Pin1, motor3Pin2, enable3Pin, 0, 0);
  moveMotor(motor4Pin1, motor4Pin2, enable4Pin, 0, 0);
  delay(1000);

  // Move all motors backward
  Serial.println("Moving Backward");
  moveMotor(motor1Pin1, motor1Pin2, enable1Pin, -1, dutyCycle);
  moveMotor(motor2Pin1, motor2Pin2, enable2Pin, -1, dutyCycle);
  moveMotor(motor3Pin1, motor3Pin2, enable3Pin, -1, dutyCycle);
  moveMotor(motor4Pin1, motor4Pin2, enable4Pin, -1, dutyCycle);
  delay(2000);

  // Stop all motors
  Serial.println("Motors Stopped");
  moveMotor(motor1Pin1, motor1Pin2, enable1Pin, 0, 0);
  moveMotor(motor2Pin1, motor2Pin2, enable2Pin, 0, 0);
  moveMotor(motor3Pin1, motor3Pin2, enable3Pin, 0, 0);
  moveMotor(motor4Pin1, motor4Pin2, enable4Pin, 0, 0);
  delay(1000);
}
