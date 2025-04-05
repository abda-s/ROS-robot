#include <Arduino.h>

class Encoder {
private:
  uint8_t pinA_;
  uint8_t pinB_;
  volatile int counter_;
  volatile int aLastState_;

public:
  Encoder(uint8_t pinA, uint8_t pinB)
    : pinA_(pinA), pinB_(pinB), counter_(0), aLastState_(0) {
    pinMode(pinA_, INPUT_PULLUP);  // Or INPUT depending on your wiring
    pinMode(pinB_, INPUT_PULLUP);
    aLastState_ = digitalRead(pinA_);
  }

  void update() {
    int aState = digitalRead(pinA_);
    if (aState != aLastState_) {
      if (digitalRead(pinB_) != aState) {
        counter_++;
      } else {
        counter_--;
      }
    }
    aLastState_ = aState;
  }

  volatile int getCounter() const {
    return counter_;
  }

  uint8_t getPinA() const {
    return pinA_;
  }
};

// Define 4 encoder objects
Encoder encoder0(17, 16);
Encoder encoder1(39, 36);
Encoder encoder2(26, 27);
Encoder encoder3(35, 34);

// Individual ISR handlers for each encoder
void IRAM_ATTR handleEncoder0() {
  encoder0.update();
}
void IRAM_ATTR handleEncoder1() {
  encoder1.update();
}
void IRAM_ATTR handleEncoder2() {
  encoder2.update();
}
void IRAM_ATTR handleEncoder3() {
  encoder3.update();
}

void setup() {
  Serial.begin(9600);

  // Attach interrupts for each encoder
  attachInterrupt(digitalPinToInterrupt(encoder0.getPinA()), handleEncoder0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1.getPinA()), handleEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2.getPinA()), handleEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder3.getPinA()), handleEncoder3, CHANGE);
}

void loop() {
  static int lastCounters[4] = { 0, 0, 0, 0 };
  int currentCounters[4];

  // Read all counters atomically
  noInterrupts();
  currentCounters[0] = encoder0.getCounter();
  currentCounters[1] = encoder1.getCounter();
  currentCounters[2] = encoder2.getCounter();
  currentCounters[3] = encoder3.getCounter();
  interrupts();

  // Check and print changes for each encoder
  for (int i = 0; i < 4; i++) {
    if (currentCounters[i] != lastCounters[i]) {
      Serial.print("Encoder ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(currentCounters[i]);
      lastCounters[i] = currentCounters[i];
    }
  }

  delay(10);  // Optional delay to reduce serial output
}