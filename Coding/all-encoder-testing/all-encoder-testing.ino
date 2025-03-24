#include <Arduino.h>

class Encoder {
private:
  int pinA;        // Pin connected to output A of the encoder
  int pinB;        // Pin connected to output B of the encoder
  int aState;      // Current state of output A
  int aLastState;  // Previous state of output A
  int counter;     // Tracks the encoder's position

public:
  // Constructor initializes pins and reads initial state
  Encoder(int aPin, int bPin)
    : pinA(aPin), pinB(bPin), counter(0) {
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
    aLastState = digitalRead(pinA);
  }

  // Updates the encoder state and returns true if position changed
  bool update() {
    aState = digitalRead(pinA);
    bool changed = false;

    if (aState != aLastState) {
      // Determine direction based on output B's state
      if (digitalRead(pinB) != aState) {
        counter++;
      } else {
        counter--;
      }
      changed = true;
    }
    aLastState = aState;
    return changed;
  }

  // Returns the current position of the encoder
  int getPosition() const {
    return counter;
  }
};

// Create four encoder instances with their respective pins (adjust pins as needed)
// yellow = c1
// green = c2

Encoder encoder1(17, 16); //c1, c2
Encoder encoder2(39, 36); //c1, c2
Encoder encoder3(26, 27); //c1, c2
Encoder encoder4(35, 34); //c1, c2

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Check and update each encoder
  Encoder* encoders[] = { &encoder1, &encoder2, &encoder3, &encoder4 };

  for (int i = 0; i < 4; i++) {
    if (encoders[i]->update()) {
      Serial.print("Encoder ");
      Serial.print(i + 1);
      Serial.print(" Position: ");
        Serial.println(encoders[i]->getPosition());
    }
  }
}